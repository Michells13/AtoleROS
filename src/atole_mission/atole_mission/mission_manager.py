"""mission_manager — secuencia automática de cosecha (acción /atole/mission/harvest).

Ciclo (empieza y termina en Home2):
    HOME2 → DETECT (estimate_pods EtH) → SELECT (pod_selector, IK de frente) → EIH_REFINE (Fase 5)
    → PREPICK → PICK → CLOSE → ROTATE → RETREAT → RELEASE → HOME2 …
- Modo "todos": repite hasta que no quedan pods alcanzables. Modo "siguiente": un pod.
- Pods ya intentados (a menos de ATTEMPTED_RADIUS_M) no se vuelven a elegir.
- Paso a paso (goal.step_mode o /atole/mission/set_step_mode): antes de cada movimiento del robot se
  espera /atole/mission/confirm_step.
- /atole/mission/abort o cancelar el goal: se cancela el movimiento en curso y se para el robot.
- Un fallo de movimiento o del gripper para el robot donde está y deja la misión en ERROR; no se
  mueve el robot por su cuenta después de un fallo.
Movimientos y velocidades como PozoleV3 (Harvest/*): Home2 y Release con moveJ; pre-pick con
MovePrepick (moveJ_IK a 3× SpeedApproach, mínimo 0.1 rad/s); pick con MovePick a SpeedPick; retirada
con MoveRetreat a SpeedApproach. Poses de pick según atole_common.grasp.
"""
import math
import threading

import numpy as np
from atole_interfaces.action import EstimatePods, GripperCommand, Harvest, MoveJoints, MovePose
from atole_interfaces.msg import ArmState, MissionStatus, SystemHealth
from atole_interfaces.srv import AcquireControl, SelectPod
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from atole_common import grasp
from atole_common.config_view import ConfigView
from atole_common.qos import LATCHED
from atole_common.stubs import run_node
from atole_common.sync_calls import CallError, call, run_action

CLIENT_ID = 'mission'
ATTEMPTED_RADIUS_M = 0.05
G = GripperCommand.Goal


class Aborted(Exception):
    pass


class MissionError(Exception):
    pass


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        group = ReentrantCallbackGroup()
        self.config = ConfigView(self)
        self.health = self.arm = None
        self.busy = False
        self._gh = None
        self.skip_gripper = False
        self.attempted = []
        self.cancel = threading.Event()
        self.confirm = threading.Event()
        self.status = MissionStatus(state='IDLE', message='listo')
        self.create_subscription(SystemHealth, '/atole/system/health', lambda m: setattr(self, 'health', m), LATCHED)
        self.create_subscription(ArmState, '/atole/arm/state', lambda m: setattr(self, 'arm', m), LATCHED)
        self.status_pub = self.create_publisher(MissionStatus, '/atole/mission/status', LATCHED)
        self.estimate = ActionClient(self, EstimatePods, '/atole/perception/estimate_pods', callback_group=group)
        self.move_joints = ActionClient(self, MoveJoints, '/atole/arm/move_joints', callback_group=group)
        self.move_pose = ActionClient(self, MovePose, '/atole/arm/move_pose', callback_group=group)
        self.gripper = ActionClient(self, GripperCommand, '/atole/gripper/command', callback_group=group)
        self.select = self.create_client(SelectPod, '/atole/task_planning/select_pod', callback_group=group)
        self.acquire = self.create_client(AcquireControl, '/atole/arm/acquire_control', callback_group=group)
        self.arm_stop = self.create_client(Trigger, '/atole/arm/stop', callback_group=group)
        ActionServer(self, Harvest, '/atole/mission/harvest', execute_callback=self._execute,
                     goal_callback=lambda _g: GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT,
                     cancel_callback=self._on_cancel, callback_group=group)
        self.create_service(Trigger, '/atole/mission/confirm_step', self._srv_confirm, callback_group=group)
        self.create_service(Trigger, '/atole/mission/abort', self._srv_abort, callback_group=group)
        self.create_service(SetBool, '/atole/mission/set_step_mode', self._srv_step_mode, callback_group=group)
        self.create_timer(1.0, self._publish, callback_group=group)
        self._publish()
        self.get_logger().info('mission_manager listo')

    # ───────────────────────── estado ─────────────────────────
    def _publish(self):
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(self.status)

    def _set(self, state=None, message=None, **fields):
        if state:
            self.status.state = state
        if message is not None:
            self.status.message = message
            self.get_logger().info(f'[{self.status.state}] {message}')
        for k, v in fields.items():
            setattr(self.status, k, v)
        self._publish()
        if self._gh is not None:
            self._gh.publish_feedback(Harvest.Feedback(status=self.status))

    # ───────────────────────── servicios ─────────────────────────
    def _on_cancel(self, _gh):
        self.cancel.set()
        return CancelResponse.ACCEPT

    def _srv_confirm(self, _req, res):
        if not self.status.awaiting_confirmation:
            res.success, res.message = False, 'no hay ningún paso esperando confirmación'
        else:
            self.confirm.set()
            res.success, res.message = True, f'confirmado: {self.status.confirmation_prompt}'
        return res

    def _srv_abort(self, _req, res):
        if not self.busy:
            res.success, res.message = False, 'no hay ninguna misión en curso'
            return res
        self.cancel.set()
        res.success, res.message = True, 'abortando la misión: se para el robot'
        return res

    def _srv_step_mode(self, req, res):
        self._set(step_mode=bool(req.data))
        res.success, res.message = True, f'paso a paso {"activado" if req.data else "desactivado"}'
        return res

    # ───────────────────────── utilidades ─────────────────────────
    def _check_cancel(self):
        if self.cancel.is_set():
            raise Aborted('misión abortada')

    def _sleep(self, seconds):
        if self.cancel.wait(max(0.0, seconds)):
            raise Aborted('misión abortada')

    def _gate(self, prompt):
        """En paso a paso, espera la confirmación del usuario antes de mover el robot."""
        self._check_cancel()
        if not self.status.step_mode:
            return
        self.confirm.clear()
        self._set(message=f'esperando confirmación: {prompt}', awaiting_confirmation=True, confirmation_prompt=prompt)
        try:
            while not self.confirm.wait(0.1):
                self._check_cancel()
        finally:
            self._set(awaiting_confirmation=False, confirmation_prompt='')

    def _h(self, key, default):
        return self.config.get_float(f'Harvest/{key}', default)

    def _joints(self, pose):
        section = self.config.section(f'Poses/{pose}')
        try:
            return [math.radians(float(section[f'J{i}'])) for i in range(1, 7)]
        except (KeyError, ValueError):
            raise MissionError(f'la pose {pose} no está guardada en Config.xml')

    def _move_to(self, pose, speed):
        target = self._joints(pose)
        current = list(self.arm.joints) if self.arm else target
        span = max(abs(a - b) for a, b in zip(target[:5], current[:5]))       # J6 va bloqueado o casi quieto
        timeout = max(self._h('MoveTimeoutS', 30.0), span / max(speed, 1e-3) * 1.5 + 5.0)
        goal = MoveJoints.Goal(client_id=CLIENT_ID, joints=target, speed=speed, accel=self._h('Accel', 0.1),
                               timeout_s=timeout)
        self._run(self.move_joints, goal, timeout + 10.0, f'moveJ a {pose}')

    def _move_cartesian(self, what, pos, quat, move_type, speed_l):
        if move_type == 'moveL':
            start = self.arm.tcp_pose.position if self.arm else None
            dist = float(np.linalg.norm(pos - [start.x, start.y, start.z])) if start else 0.5
            speed, accel, mt = speed_l, self._h('Accel', 0.1), MovePose.Goal.MOVE_L
            timeout = max(self._h('MoveTimeoutS', 30.0), dist / max(speed, 1e-3) * 1.5 + 5.0)
        else:                                     # moveJ_IK como PozoleV3: 3× en articulaciones
            speed, accel, mt = max(0.1, 3 * speed_l), max(0.2, 3 * self._h('Accel', 0.1)), MovePose.Goal.MOVE_J_IK
            timeout = max(self._h('MoveTimeoutS', 30.0), 60.0)
        goal = MovePose.Goal(client_id=CLIENT_ID, target=grasp.pose_stamped(pos, quat), move_type=mt, speed=speed,
                             accel=accel, timeout_s=timeout, prefer_facing=True)
        self._run(self.move_pose, goal, timeout + 10.0, f'{move_type} al {what}')

    def _run(self, client, goal, timeout, what):
        try:
            res = run_action(client, goal, timeout, cancel=self.cancel)
        except CallError as e:
            if self.cancel.is_set():
                raise Aborted('misión abortada')
            raise MissionError(f'{what}: {e}')
        if not res.ok:
            raise MissionError(f'{what}: {res.message}')
        return res

    def _grip(self, what, command, **kw):
        if self.skip_gripper:
            self._set(message=f'{what}: gripper omitido (skip_gripper)')
            return
        self._run(self.gripper, G(command=command, timeout_s=15.0, **kw), 25.0, what)

    # ───────────────────────── pasos ─────────────────────────
    def _home2(self, speed):
        self._set('HOME2', 'a Home2')
        self._gate('mover a Home2')
        self._grip('abrir gripper', G.OPEN, force=30, speed=50)
        self._grip('centrar rotación', G.ROTATE, angle_deg=0, force=30, speed=50)
        self._move_to('Home2', speed)

    def _detect(self):
        self._set('DETECT', 'estimando pods con la cámara EtH')
        stages = lambda f: self._set(message=f'estimate_pods: {f.stage}' + (f' {f.done}/{f.total}' if f.total else ''))
        try:
            res = run_action(self.estimate, EstimatePods.Goal(), 300.0, feedback=stages, cancel=self.cancel)
        except CallError as e:
            if self.cancel.is_set():
                raise Aborted('misión abortada')
            raise MissionError(f'estimate_pods: {e}')
        if not res.ok:
            raise MissionError(f'estimate_pods: {res.message}')
        self._set(message=res.message)
        return res.pods

    def _select(self, pods, strategy):
        for p in pods.pods:
            g = np.array([p.grasp_bottom.x, p.grasp_bottom.y, p.grasp_bottom.z])
            if p.status == 'ok' and any(np.linalg.norm(g - a) < ATTEMPTED_RADIUS_M for a in self.attempted):
                p.status = 'already_attempted'
        self._set('SELECT', 'eligiendo pod (IK de frente)')
        try:
            res = call(self.select, SelectPod.Request(pods=pods, strategy=strategy), timeout=60.0)
        except CallError as e:
            raise MissionError(f'select_pod: {e}')
        if not res.ok:
            raise MissionError(f'select_pod: {res.message}')
        reachable = sum(p.reachable for p in res.evaluated.pods)
        self._set(message=res.message, pods_remaining=reachable)
        return (res.evaluated.pods[res.selected_index] if res.selected_index >= 0 else None), reachable

    def _harvest_pod(self, pod, eih_strategy):
        self._set('EIH_REFINE', f'EiH ({eih_strategy}) llega en la Fase 5: se usa la pose EtH', current_pod_id=pod.id)
        t = grasp.targets(pod, self.config.section('Harvest'))
        speed_approach = self._h('SpeedApproach', 0.15)
        self._set('PREPICK', f'pre-pick del pod {pod.id}')
        self._gate(f'pre-pick del pod {pod.id}')
        self._move_cartesian('pre-pick', *t['pre'], self.config.get('Harvest/MovePrepick', 'moveJ_IK'), speed_approach)
        self._sleep(self._h('StepDelayS', 1.0))
        self._set('PICK', f'pick del pod {pod.id}')
        self._gate(f'pick del pod {pod.id}')
        self._move_cartesian('pick', *t['pick'], self.config.get('Harvest/MovePick', 'moveL'), self._h('SpeedPick', 0.05))
        self._set('CLOSE', 'cerrando el gripper')
        self._grip('cerrar gripper', G.CLOSE)
        self._sleep(self._h('StepDelayS', 1.0))
        angle = round(self._h('RotateDeg', 180.0)) * (-1 if self.config.get('Harvest/RotateDir', 'cw') == 'ccw' else 1)
        self._set('ROTATE', f'girando el gripper {angle}°')
        self._grip('girar gripper', G.ROTATE, angle_deg=angle)
        self._sleep(self._h('RotateDelayS', 1.8))
        self._set('RETREAT', 'retirada al pre-pick')
        self._gate('retirada al pre-pick')
        self._move_cartesian('pre-pick (retirada)', *t['pre'], self.config.get('Harvest/MoveRetreat', 'moveL'), speed_approach)
        self._set('RELEASE', 'a la pose Release')
        self._gate('mover a Release')
        self._move_to('Release', self._h('SpeedRelease', 0.20))
        self._grip('abrir gripper', G.OPEN, force=30, speed=50)
        self._grip('centrar rotación', G.ROTATE, angle_deg=0, force=30, speed=50)

    # ───────────────────────── acción ─────────────────────────
    def _execute(self, gh):
        g = gh.request
        self.busy, self._gh = True, gh
        self.cancel.clear()
        self.attempted, self.skip_gripper = [], g.skip_gripper
        harvested = failed = 0
        mode = 'next' if g.mode == Harvest.Goal.MODE_NEXT else 'all'
        eih = g.eih_strategy or self.config.get('EIH/Strategy', 'single')
        self.status = MissionStatus(state='IDLE', mode=mode, step_mode=g.step_mode, eih_strategy=eih, current_pod_id=-1)
        result = Harvest.Result()
        owned = False
        try:
            if not (self.health and self.health.ready):
                bad = [f'{i.name}: {i.message}' for i in (self.health.items if self.health else []) if i.level not in (0, 3)]
                raise MissionError('el sistema no está READY' + (f' ({"; ".join(bad)})' if bad else ''))
            ack = call(self.acquire, AcquireControl.Request(client_id=CLIENT_ID), timeout=5.0)
            if not ack.ok:
                raise MissionError(f'no se pudo tomar el control del robot: {ack.message}')
            owned = True
            while True:
                self._home2(self._h('SpeedApproach', 0.15) if harvested == 0 else self._h('SpeedRelease', 0.20))
                pod, _reachable = self._select(self._detect(), g.selection_strategy)
                if pod is None:
                    self._set(message='no quedan pods alcanzables')
                    break
                self.attempted.append(np.array([pod.grasp_bottom.x, pod.grasp_bottom.y, pod.grasp_bottom.z]))
                try:
                    self._harvest_pod(pod, eih)
                except MissionError:
                    failed += 1
                    raise
                harvested += 1
                self._set(pods_done=harvested)
                if mode == 'next':
                    self._home2(self._h('SpeedRelease', 0.20))
                    break
            result.ok, result.message = True, f'{harvested} pods cosechados'
            self._set('IDLE', f'misión terminada: {result.message}', current_pod_id=-1)
            gh.succeed()
        except Aborted as e:
            self._stop_robot()
            result.ok, result.message = False, str(e)
            self._set('IDLE', f'{e}: robot parado donde estaba')
            gh.canceled() if gh.is_cancel_requested else gh.abort()
        except (MissionError, CallError) as e:
            self._stop_robot()
            result.ok, result.message = False, str(e)
            self._set('ERROR', f'{e} — robot parado; revisa antes de relanzar')
            gh.abort()
        finally:
            if owned:
                try:
                    call(self.acquire, AcquireControl.Request(client_id=CLIENT_ID, release=True), timeout=5.0)
                except CallError:
                    pass
            self._set(awaiting_confirmation=False, confirmation_prompt='', pods_failed=failed)
            self.busy, self._gh = False, None
        result.pods_harvested, result.pods_failed = harvested, failed
        return result

    def _stop_robot(self):
        try:
            call(self.arm_stop, Trigger.Request(), timeout=5.0)
        except CallError as e:
            self.get_logger().error(f'no se pudo parar el robot: {e}')


def main(args=None):
    run_node(MissionManager, args)


if __name__ == '__main__':
    main()
