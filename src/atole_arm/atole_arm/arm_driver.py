"""arm_driver — único nodo que habla con el AUBO iS10.

- Publica /joint_states y /atole/arm/state a Robot/StateRateHz (20 Hz por defecto) y el TF
  tool0→tcp a partir del offset de herramienta del controlador.
- Acciones cancelables /atole/arm/move_joints y /atole/arm/move_pose. Los movimientos se
  envían sin bloquear y el nodo vigila: cancelación → stop, protective stop o emergencia →
  aborto, timeout → stop, llegada = robot en reposo y dentro de la tolerancia.
- Lease de movimiento (acquire_control): solo el dueño mueve el robot; stop lo acepta siempre.
- backend: mock (simulado en memoria) | vm (Robot/Aubo) | real (Robot/AuboReal).

Unidades de las acciones: move_joints y MOVE_J_IK usan rad/s y rad/s²; MOVE_L usa m/s y m/s².
"""
import math
import threading
import time

from atole_interfaces.action import MoveJoints, MovePose
from atole_interfaces.msg import ArmState
from atole_interfaces.srv import (AcquireControl, ComputeFk, ComputeIk, ConfigSet, JogJoint,
                                  SetFreedrive)
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger


from atole_arm.aubo_client import AuboClient, AuboError, MockAubo
from atole_arm.poses import (aubo_to_pose, facing_offset_deg, pick_facing, pose_to_aubo,
                             rotate_about_tool_z, wrap, FACING_MAX_DEG)
from atole_common.config_view import ConfigView
from atole_common.static_tf import StaticTf
from atole_common.qos import LATCHED
from atole_common.stubs import run_node

JOINT_NAMES = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
               'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
JOINT_TOL = math.radians(0.5)
POS_TOL_M = 0.001
DEFAULT_JOINT_SPEED, DEFAULT_JOINT_ACCEL = 0.5, 0.5       # rad/s, rad/s²
DEFAULT_LINE_SPEED, DEFAULT_LINE_ACCEL = 0.1, 0.2         # m/s, m/s²
STALL_S = 1.5          # en reposo sin haber llegado durante este tiempo = movimiento interrumpido


def canonical_joints(target, keep=()):
    """Cada articulación a su ángulo equivalente en [−180°, 180°] (misma pose física), salvo los
    índices de `keep`. El AUBO (VM 0.23.1) acepta un moveJoint con algún objetivo fuera de ±180°
    —p. ej. J5 = 292.6° o J4 = −227.2°— y no lo ejecuta, sin error, aunque getJointMaxPositions diga
    ±360°. Con el valor equivalente dentro de ±180° sí se mueve, sea cual sea el recorrido."""
    return [t if i in keep else wrap(t) for i, t in enumerate(target)]
RECONNECT_S = 5.0


class ArmDriver(Node):

    def __init__(self):
        super().__init__('arm_driver')
        self.backend = self.declare_parameter('backend', 'mock').value
        self.client = None
        self.state = None
        self.state_seq = 0
        self.owner = ''
        self.lock_j6 = True
        self.busy = False
        self.last_fault = ''
        self._read_failures = 0
        self._tcp_offset = None
        self._connecting = False
        self._prev_safety = None
        self._stop_event = threading.Event()   # stop recibido durante un movimiento
        self._slow = {}            # modo, offset del TCP y freedrive (se leen a 2 Hz)
        self._cycle = 0

        state_group = MutuallyExclusiveCallbackGroup()
        cmd_group = ReentrantCallbackGroup()
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_pub = self.create_publisher(ArmState, '/atole/arm/state', LATCHED)
        self.fault_pub = self.create_publisher(ArmState, '/atole/arm/fault', LATCHED)
        self.tf_static = StaticTf(self)
        self.config_set = self.create_client(ConfigSet, '/atole/config/set', callback_group=cmd_group)

        for action_type, name, execute in ((MoveJoints, 'move_joints', self._exec_joints),
                                           (MovePose, 'move_pose', self._exec_pose)):
            ActionServer(self, action_type, f'/atole/arm/{name}', execute_callback=execute,
                         goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=cmd_group)
        services = [
            (Trigger, 'stop', self._srv_stop), (Trigger, 'unlock_pstop', self._srv_unlock),
            (Trigger, 'reconnect', self._srv_reconnect), (SetBool, 'set_j6_lock', self._srv_j6_lock),
            (SetFreedrive, 'set_freedrive', self._srv_freedrive), (ComputeIk, 'compute_ik', self._srv_ik),
            (ComputeFk, 'compute_fk', self._srv_fk), (JogJoint, 'jog_joint', self._srv_jog),
            (AcquireControl, 'acquire_control', self._srv_acquire),
        ]
        for srv_type, name, callback in services:
            self.create_service(srv_type, f'/atole/arm/{name}', callback, callback_group=cmd_group)

        self.config = ConfigView(self, on_update=self._on_config)
        self.create_timer(1.0 / 20.0, self._update_state, callback_group=state_group)
        self.get_logger().info(f'arm_driver arrancado (backend={self.backend}); esperando /atole/config')

    # ───────────────────────── conexión y estado ─────────────────────────
    def _on_config(self, cfg):
        self.lock_j6 = cfg.get_bool('Robot/LockJ6', True)
        if self.client is None:
            if self.backend == 'mock':
                self.client = MockAubo()
                self.get_logger().info('robot simulado (mock) listo')
            else:
                section = 'Robot/AuboReal' if self.backend == 'real' else 'Robot/Aubo'
                c = cfg.section(section)
                self.client = AuboClient(c.get('IP', ''), c.get('Port', '30004'), c.get('User', 'aubo'),
                                         c.get('Pass', ''), c.get('RobotName', ''))
                self._start_connect()

    def _start_connect(self):
        if self._connecting or isinstance(self.client, MockAubo):
            return
        self._connecting = True
        threading.Thread(target=self._connect_loop, daemon=True).start()

    def _connect_loop(self):
        while rclpy_ok():
            try:
                name = self.client.connect()
                self._read_failures = 0
                self.get_logger().info(f'conectado al AUBO {self.client.ip}:{self.client.port} (robot "{name}")')
                break
            except Exception as e:
                self.get_logger().warn(f'no se pudo conectar a {self.client.ip}: {e}; reintento en {RECONNECT_S:.0f} s',
                                       throttle_duration_sec=30.0)
                time.sleep(RECONNECT_S)
        self._connecting = False

    def _update_state(self):
        if self.client is None or not self.client.connected:
            self._publish_state(None)
            return
        try:
            slow_cycle = self._cycle % 10 == 0 or not self._slow
            self._cycle += 1
            s = self.client.read(slow=slow_cycle)
            if slow_cycle:
                self._slow = {k: s[k] for k in ('robot_mode', 'tcp_offset', 'freedrive')}
            s.update(self._slow)
            self._read_failures = 0
        except Exception as e:
            self._read_failures += 1
            if self._read_failures == 3:
                self._fault(f'conexión perdida con el robot: {e}')
                self.client.disconnect()
                self._start_connect()
            return
        self.state, self.state_seq = s, self.state_seq + 1
        if s['safety_mode'] != self._prev_safety:
            if s['protective_stop'] or s['emergency_stop']:
                self._fault(f'seguridad: {s["safety_mode"]}')
            self._prev_safety = s['safety_mode']
        if self._tcp_offset is None or any(abs(a - b) > 1e-6 for a, b in zip(s['tcp_offset'], self._tcp_offset)):
            self._tcp_offset = list(s['tcp_offset'])
            self._publish_tcp_tf(self._tcp_offset)
        now = self.get_clock().now().to_msg()
        js = JointState(name=JOINT_NAMES, position=list(s['joints']))
        js.header.stamp = now
        self.joint_pub.publish(js)
        self._publish_state(s, now)

    def _publish_state(self, s, stamp=None):
        msg = ArmState(connected=s is not None, j6_locked=self.lock_j6, control_owner=self.owner,
                       last_fault=self.last_fault)
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        if s is not None:
            msg.joints = list(s['joints'])
            msg.tcp_pose = aubo_to_pose(s['tcp'])
            msg.tcp_pose_native = list(s['tcp'])
            msg.tcp_offset_native = list(s['tcp_offset'])
            msg.is_moving = not (s['steady'] and s['queue'] == 0 and s['exec_id'] == -1)
            msg.freedrive = s['freedrive']
            msg.robot_mode, msg.safety_mode = s['robot_mode'], s['safety_mode']
            msg.protective_stop, msg.emergency_stop = s['protective_stop'], s['emergency_stop']
        self.state_pub.publish(msg)
        return msg

    def _publish_tcp_tf(self, offset):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'tool0', 'tcp'
        p = aubo_to_pose(offset)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            p.position.x, p.position.y, p.position.z
        t.transform.rotation = p.orientation
        self.tf_static.sendTransform(t)

    def _fault(self, text):
        self.last_fault = text
        self.get_logger().error(text)
        self.fault_pub.publish(self._publish_state(self.state))

    # ───────────────────────── comprobaciones ─────────────────────────
    def _cannot_move(self, client_id):
        if self.client is None or not self.client.connected or self.state is None:
            return 'robot desconectado'
        if self.owner and self.owner != client_id:
            return f'el control del robot lo tiene "{self.owner}"'
        s = self.state
        if s['emergency_stop'] or s['protective_stop']:
            return f'robot en {s["safety_mode"]}'
        if s['robot_mode'] != 'Running':
            return f'robot en modo {s["robot_mode"]} (se necesita Running)'
        if s['freedrive']:
            return 'freedrive activado'
        return None

    def _fresh_state(self, timeout=1.0):
        seq, t0 = self.state_seq, time.monotonic()
        while self.state_seq == seq and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        return self.state

    def _on_goal(self, _goal):
        return GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT

    def _timeout(self, requested):
        return requested if requested > 0 else self.config.get_float('Harvest/MoveTimeoutS', 30.0)

    # ───────────────────────── acciones ─────────────────────────
    def _exec_joints(self, gh):
        g, res = gh.request, MoveJoints.Result()
        self.busy = True
        try:
            error = self._cannot_move(g.client_id)
            if error:
                return self._abort(gh, res, error)
            target = list(g.joints)
            if self.lock_j6 and not g.bypass_j6_lock:
                target[5] = self.state['joints'][5]
            near = canonical_joints(target, keep=(5,) if self.lock_j6 and not g.bypass_j6_lock else ())
            changed = [f'J{i + 1} {math.degrees(a):.1f}° → {math.degrees(b):.1f}°'
                       for i, (a, b) in enumerate(zip(target, near)) if abs(a - b) > 1e-6]
            if changed:
                self.get_logger().info(f'move_joints: ángulo equivalente dentro de ±180°: {", ".join(changed)}')
            target = near
            try:
                self.client.move_joint(target, g.speed or DEFAULT_JOINT_SPEED, g.accel or DEFAULT_JOINT_ACCEL)
            except AuboError as e:
                return self._abort(gh, res, str(e))

            def distance(s):
                return max(abs(wrap(a - b)) for a, b in zip(s['joints'], target))

            def feedback(s, d):
                return MoveJoints.Feedback(joints=list(s['joints']), remaining_rad=d)
            return self._watch(gh, res, distance, JOINT_TOL, self._timeout(g.timeout_s), feedback)
        finally:
            self.busy = False

    def _exec_pose(self, gh):
        g, res = gh.request, MovePose.Result()
        self.busy = True
        try:
            error = self._cannot_move(g.client_id)
            if error:
                return self._abort(gh, res, error)
            if g.target.header.frame_id not in ('', 'base_link'):
                return self._abort(gh, res, f'frame_id "{g.target.header.frame_id}" no soportado (usa base_link)')
            pose = pose_to_aubo(g.target.pose)
            seed = list(self.state['joints'])
            try:
                if g.move_type == MovePose.Goal.MOVE_J_IK:
                    q, why = self._solve_ik(seed, pose, g.prefer_facing)
                    if self.lock_j6:
                        q[5] = seed[5]
                    q = canonical_joints(q, keep=(5,) if self.lock_j6 else ())
                    res.ik_solution = q
                    self.get_logger().info(f'move_pose J_IK: {why}; J={[round(math.degrees(v), 1) for v in q]}°')
                    self.client.move_joint(q, g.speed or DEFAULT_JOINT_SPEED, g.accel or DEFAULT_JOINT_ACCEL)

                    def distance(s):
                        return max(abs(wrap(a - b)) for a, b in zip(s['joints'], q))
                    tol = JOINT_TOL
                else:
                    if self.lock_j6:
                        pose = self._keep_j6_linear(seed, pose)
                    self.client.move_line(pose, g.speed or DEFAULT_LINE_SPEED, g.accel or DEFAULT_LINE_ACCEL)

                    def distance(s):
                        return math.dist(s['tcp'][:3], pose[:3])
                    tol = POS_TOL_M
            except AuboError as e:
                return self._abort(gh, res, str(e))

            def feedback(s, d):
                return MovePose.Feedback(tcp_pose=aubo_to_pose(s['tcp']), remaining_m=math.dist(s['tcp'][:3], pose[:3]))
            return self._watch(gh, res, distance, tol, self._timeout(g.timeout_s), feedback)
        finally:
            self.busy = False

    def _solve_ik(self, seed, pose, prefer_facing):
        q = self.client.ik(seed, pose)
        if not prefer_facing:
            return q, 'solución con semilla'
        off = facing_offset_deg(q, pose)
        if off is None or off <= FACING_MAX_DEG:
            return q, 'solución con semilla (de frente)'
        return pick_facing(q, self.client.ik_all(pose), seed, pose)

    def _keep_j6_linear(self, seed, pose):
        """Con J6 bloqueado, gira el objetivo alrededor del eje de la brida para que la IK
        conserve J6 y el movimiento siga siendo una línea recta (moveLine)."""
        q = self.client.ik(seed, pose)
        delta = wrap(seed[5] - q[5])
        if abs(delta) < JOINT_TOL:
            return pose
        adjusted = rotate_about_tool_z(pose, delta)
        check = self.client.ik(seed, adjusted)
        residual = math.degrees(abs(wrap(check[5] - seed[5])))
        self.get_logger().info(f'J6 bloqueado: objetivo girado {math.degrees(delta):.1f}° sobre Z de la brida '
                               f'(J6 residual {residual:.2f}°)')
        return adjusted

    def _watch(self, gh, res, distance, tol, timeout, feedback):
        """Vigila el movimiento en curso hasta llegar, cancelarse, fallar o agotar el tiempo."""
        t0, stalled_since = time.monotonic(), None
        self._stop_event.clear()
        s = self._fresh_state()
        start, moved = list(s['joints']), False
        while True:
            if self._stop_event.is_set():
                return self._abort(gh, res, 'detenido por /atole/arm/stop')
            if gh.is_cancel_requested:
                self.client.stop()
                gh.canceled()
                return self._result(res, False, 'cancelado')
            if s['protective_stop'] or s['emergency_stop']:
                return self._abort(gh, res, f'movimiento interrumpido: robot en {s["safety_mode"]}')
            d = distance(s)
            moved = moved or max(abs(a - b) for a, b in zip(s['joints'], start)) > JOINT_TOL
            gh.publish_feedback(feedback(s, d))
            idle = s['steady'] and s['queue'] == 0 and s['exec_id'] == -1
            if idle and d <= tol:
                gh.succeed()
                return self._result(res, True, 'llegó al objetivo')
            if idle:
                stalled_since = stalled_since or time.monotonic()
                if time.monotonic() - stalled_since > STALL_S:
                    if not moved:
                        return self._abort(gh, res, 'el AUBO aceptó el movimiento pero no lo ejecutó '
                                                    f'(error {d:.4f}); revisa el objetivo')
                    return self._abort(gh, res, f'el robot se detuvo sin llegar (error {d:.4f})')
            else:
                stalled_since = None
            if time.monotonic() - t0 > timeout:
                self.client.stop()
                return self._abort(gh, res, f'timeout de {timeout:.0f} s')
            time.sleep(0.05)
            s = self._fresh_state()

    def _result(self, res, ok, message):
        res.ok, res.message = ok, message
        res.final_state = self._publish_state(self.state)
        return res

    def _abort(self, gh, res, message):
        self.get_logger().warn(f'movimiento abortado: {message}')
        gh.abort()
        return self._result(res, False, message)

    # ───────────────────────── servicios ─────────────────────────
    def _srv_stop(self, _req, res):
        try:
            self.client.stop()
            self._stop_event.set()
            res.success, res.message = True, 'stop enviado'
        except Exception as e:
            res.success, res.message = False, f'stop falló: {e}'
        return res

    def _srv_unlock(self, _req, res):
        try:
            code = self.client.unlock_protective_stop()
            res.success = self.client.ok(code)
            res.message = 'protective stop desbloqueado' if res.success else f'desbloqueo rechazado (código {code})'
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def _srv_reconnect(self, _req, res):
        if isinstance(self.client, MockAubo) or self.client is None:
            res.success, res.message = self.client is not None, 'nada que reconectar'
            return res
        self.client.disconnect()
        self._start_connect()
        res.success, res.message = True, 'reconectando'
        return res

    def _srv_j6_lock(self, req, res):
        self.lock_j6 = bool(req.data)
        if self.config_set.service_is_ready():
            self.config_set.call_async(ConfigSet.Request(key='Robot/LockJ6', value=str(self.lock_j6).lower(), persist=True))
        res.success, res.message = True, f'bloqueo de J6 {"activado" if self.lock_j6 else "desactivado"}'
        return res

    def _srv_freedrive(self, req, res):
        if req.enable:
            error = self._cannot_move(req.client_id)
            if error and not error.startswith('freedrive'):
                res.ok, res.message = False, error
                return res
        try:
            code = self.client.set_freedrive(req.enable, req.damping or 0.6)
            res.ok = self.client.ok(code)
            res.message = f'freedrive {"activado" if req.enable else "desactivado"}' if res.ok else f'código {code}'
        except Exception as e:
            res.ok, res.message = False, str(e)
        return res

    def _srv_ik(self, req, res):
        try:
            seed = list(self.state['joints']) if self.state else [0.0] * 6
            q, why = self._solve_ik(seed, pose_to_aubo(req.target.pose), req.prefer_facing)
            res.ok, res.message, res.joints = True, why, q
        except Exception as e:
            res.ok, res.message = False, str(e)
        return res

    def _srv_fk(self, req, res):
        try:
            res.tcp = PoseStamped(pose=aubo_to_pose(self.client.fk(list(req.joints))))
            res.tcp.header.frame_id = 'base_link'
            res.ok, res.message = True, 'ok'
        except Exception as e:
            res.ok, res.message = False, str(e)
        return res

    def _srv_jog(self, req, res):
        error = self._cannot_move(req.client_id) or ('hay un movimiento en curso' if self.busy else None)
        if error or req.joint_index > 5:
            res.ok, res.message = False, error or 'joint_index debe estar entre 0 y 5'
            return res
        target = list(self.state['joints'])
        target[req.joint_index] += math.radians(req.delta_deg)
        try:
            self.client.move_joint(target, req.speed or 0.3, DEFAULT_JOINT_ACCEL)
            res.ok, res.message = True, f'J{req.joint_index + 1} {req.delta_deg:+.1f}°'
        except AuboError as e:
            res.ok, res.message = False, str(e)
        return res

    def _srv_acquire(self, req, res):
        if req.release:
            if self.owner in (req.client_id, ''):
                self.owner = ''
                res.ok, res.message = True, 'control liberado'
            else:
                res.ok, res.message = False, f'el control lo tiene "{self.owner}"'
        elif self.owner in ('', req.client_id) or req.force:
            previous, self.owner = self.owner, req.client_id
            res.ok = True
            res.message = f'control para "{req.client_id}"' + (f' (quitado a "{previous}")' if previous and previous != req.client_id else '')
        else:
            res.ok, res.message = False, f'el control lo tiene "{self.owner}"'
        res.owner = self.owner
        self.get_logger().info(f'acquire_control: {res.message}')
        return res


def rclpy_ok():
    import rclpy
    return rclpy.ok()


def main(args=None):
    run_node(ArmDriver, args)


if __name__ == '__main__':
    main()
