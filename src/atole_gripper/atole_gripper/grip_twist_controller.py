"""grip_twist_controller — control del gripper RGI-100 (apertura + rotación).

- Publica /atole/gripper/state a 5 Hz con la posición y el ángulo REALES (registros de feedback).
- Acción /atole/gripper/command: OPEN, CLOSE (usa Harvest/ClosePos|CloseForce|CloseSpeed),
  MOVE (apertura 0..1000, 1000 = abierto) y ROTATE (ángulo absoluto en grados).
- Servicios: reinit (inicialización completa: MUEVE el gripper), reconnect, scan_port.
- Por defecto NO se inicializa solo al arrancar (auto_init:=false), porque inicializar lo mueve.
"""
import threading
import time

from atole_interfaces.action import GripperCommand
from atole_interfaces.msg import GripperState
from atole_interfaces.srv import ConfigSet, ScanPort
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from atole_common.config_view import ConfigView
from atole_common.qos import LATCHED
from atole_common.stubs import run_node
from atole_gripper.rgi100 import GRIP_NAMES, INIT_NAMES, GripperError, Rgi100, scan

OPEN_POSITION = 1000
DEFAULT_TIMEOUT_S = 10.0


class GripTwistController(Node):

    def __init__(self):
        super().__init__('grip_twist_controller')
        self.auto_init = self.declare_parameter('auto_init', False).value
        self.driver = None
        self.busy = False
        self.last = None
        self._connecting = False
        cmd_group = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(GripperState, '/atole/gripper/state', LATCHED)
        self.config_set = self.create_client(ConfigSet, '/atole/config/set', callback_group=cmd_group)
        ActionServer(self, GripperCommand, '/atole/gripper/command', execute_callback=self._execute,
                     goal_callback=lambda _: GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cmd_group)
        self.create_service(Trigger, '/atole/gripper/reinit', self._srv_reinit, callback_group=cmd_group)
        self.create_service(Trigger, '/atole/gripper/reconnect', self._srv_reconnect, callback_group=cmd_group)
        self.create_service(ScanPort, '/atole/gripper/scan_port', self._srv_scan, callback_group=cmd_group)
        self.config = ConfigView(self, on_update=self._on_config)
        self.create_timer(0.2, self._update_state, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'grip_twist_controller arrancado (auto_init={self.auto_init}); esperando /atole/config')

    # ───────────────────────── conexión ─────────────────────────
    def _params(self):
        g = self.config.section('Gripper')
        return {'slave': g.get('SlaveAddress', '1'), 'baud': g.get('Baud', '115200'),
                'parity': g.get('Parity', 'N'), 'stopbits': g.get('StopBits', '1'),
                'timeout': g.get('Timeout', '1.0')}

    def _on_config(self, _cfg):
        if self.driver is None and not self._connecting:
            self._connecting = True
            threading.Thread(target=self._connect, daemon=True).start()

    def _connect(self):
        port = self.config.get('Gripper/Port', '/dev/ttyUSB0')
        try:
            for candidate in (port, None):
                if candidate is None:
                    candidate = scan(**self._params())
                    if candidate is None:
                        self.get_logger().error('no se encontró el RGI-100 en ningún puerto serie')
                        return
                    self.get_logger().warn(f'el gripper no responde en {port}; encontrado en {candidate}')
                    self._persist('Gripper/Port', candidate)
                driver = Rgi100(candidate, **self._params())
                try:
                    driver.open()
                except Exception:
                    driver.close()
                    continue
                self.driver = driver
                state = driver.read_state()
                self.get_logger().info(f'RGI-100 conectado en {candidate} (estado de inicialización: '
                                       f'{INIT_NAMES.get(state["init_state"], state["init_state"])})')
                if state['init_state'] != 1:
                    if self.auto_init:
                        self._initialize()
                    else:
                        self.get_logger().warn('el gripper está sin inicializar; llama a /atole/gripper/reinit '
                                               '(ojo: inicializar lo MUEVE)')
                return
        finally:
            self._connecting = False

    def _persist(self, key, value):
        if self.config_set.service_is_ready():
            self.config_set.call_async(ConfigSet.Request(key=key, value=value, persist=True))

    def _initialize(self):
        g = self.config.section('Gripper')
        self.get_logger().info('inicializando el gripper (se mueve)...')
        self.driver.initialize(full=g.get('FullInit', '1') == '1', direction=int(g.get('InitDirection', '0')),
                               timeout=float(g.get('InitTimeoutS', '15')))
        self.get_logger().info('gripper inicializado')

    # ───────────────────────── estado ─────────────────────────
    def _update_state(self):
        msg = GripperState(connected=self.driver is not None, busy=self.busy)
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.driver is not None:
            msg.port = self.driver.port
            try:
                s = self.driver.read_state()
                self.last = s
                msg.init_state, msg.gripper_state = s['init_state'], s['grip_state']
                msg.init_state_name = INIT_NAMES.get(s['init_state'], str(s['init_state']))
                msg.gripper_state_name = GRIP_NAMES.get(s['grip_state'], str(s['grip_state']))
                msg.last_opening, msg.last_angle_deg = s['position'], s['angle']
            except GripperError as e:
                msg.connected = False
                self.get_logger().warn(f'lectura del gripper falló: {e}', throttle_duration_sec=10.0)
        self.state_pub.publish(msg)
        return msg

    # ───────────────────────── acción ─────────────────────────
    def _execute(self, gh):
        g, res = gh.request, GripperCommand.Result()
        self.busy = True
        try:
            if self.driver is None:
                return self._finish(gh, res, False, 'gripper desconectado')
            if self.driver.read_register(0x0200) != 1:
                return self._finish(gh, res, False, 'gripper sin inicializar (llama a /atole/gripper/reinit)')
            h = self.config.section('Harvest')
            C = GripperCommand.Goal
            if g.command in (C.OPEN, C.CLOSE, C.MOVE):
                if g.command == C.OPEN:
                    opening, force, speed = OPEN_POSITION, g.force or 50, g.speed or 50
                elif g.command == C.CLOSE:
                    opening = int(float(h.get('ClosePos', '0')))
                    force = g.force or int(float(h.get('CloseForce', '100')))
                    speed = g.speed or int(float(h.get('CloseSpeed', '100')))
                else:
                    opening, force, speed = g.opening, g.force or 50, g.speed or 50
                self.driver.move(opening, force, speed)
                return self._wait(gh, res, 'grip_state', g.timeout_s)
            if g.command == C.ROTATE:
                self.driver.rotate(g.angle_deg, g.force or int(float(h.get('RotateForce', '50'))),
                                   g.speed or int(float(h.get('RotateSpeed', '50'))))
                return self._wait(gh, res, 'rot_state', g.timeout_s)
            return self._finish(gh, res, False, f'comando desconocido: {g.command}')
        except GripperError as e:
            return self._finish(gh, res, False, str(e))
        finally:
            self.busy = False

    def _wait(self, gh, res, field, timeout):
        """Espera a que el gripper (o la rotación) deje de moverse."""
        timeout = timeout or DEFAULT_TIMEOUT_S
        t0 = time.monotonic()
        time.sleep(0.25)          # el estado tarda un instante en pasar a "moviendo"
        while time.monotonic() - t0 < timeout:
            if gh.is_cancel_requested:
                gh.canceled()
                return self._result(res, False, 'cancelado (el gripper termina el movimiento en curso)')
            s = self.driver.read_state()
            gh.publish_feedback(GripperCommand.Feedback(state=self._update_state()))
            if s[field] != 0:
                if field == 'grip_state':
                    text = {1: 'llegó a la posición', 2: 'agarró un objeto', 3: 'objeto caído'}[s['grip_state']]
                    return self._finish(gh, res, s['grip_state'] != 3, text)
                text = {1: 'rotación completada', 2: 'rotación bloqueada', 3: 'bloqueo liberado'}.get(s['rot_state'], '')
                return self._finish(gh, res, s['rot_state'] == 1, text)
            time.sleep(0.1)
        return self._finish(gh, res, False, f'timeout de {timeout:.0f} s')

    def _finish(self, gh, res, ok, message):
        (gh.succeed if ok else gh.abort)()
        if not ok:
            self.get_logger().warn(f'comando del gripper: {message}')
        return self._result(res, ok, message)

    def _result(self, res, ok, message):
        res.ok, res.message = ok, message
        res.final_state = self._update_state()
        return res

    # ───────────────────────── servicios ─────────────────────────
    def _srv_reinit(self, _req, res):
        if self.driver is None or self.busy:
            res.success, res.message = False, 'gripper desconectado' if self.driver is None else 'gripper ocupado'
            return res
        self.busy = True
        try:
            self._initialize()
            res.success, res.message = True, 'gripper inicializado'
        except GripperError as e:
            res.success, res.message = False, str(e)
        finally:
            self.busy = False
        return res

    def _srv_reconnect(self, _req, res):
        if self.driver is not None:
            self.driver.close()
            self.driver = None
        self._on_config(self.config)
        res.success, res.message = True, 'reconectando'
        return res

    def _srv_scan(self, _req, res):
        port = scan(**self._params())
        res.ok, res.port = port is not None, port or ''
        res.message = f'RGI-100 en {port}' if port else 'no se encontró el RGI-100'
        return res


def main(args=None):
    run_node(GripTwistController, args)


if __name__ == '__main__':
    main()
