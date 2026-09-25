"""Cliente del controlador AUBO (pyaubo_sdk) usado por arm_driver.

Todas las llamadas RPC pasan por un único lock: el nodo lee el estado a 20 Hz mientras
vigila los movimientos desde otro hilo.
Convención de poses del AUBO: [x, y, z, roll, pitch, yaw] en metros y radianes, RPY
intrínseco ZYX (R = Rz(yaw) · Ry(pitch) · Rx(roll)).
"""
import threading
import time

REQUEST_IGNORE = 13        # el controlador ignora la orden si está ocupado un instante; reintentar es seguro
ESTOP_MODES = ('RobotEmergencyStop', 'SystemEmergencyStop')
PSTOP_MODES = ('ProtectiveStop', 'SafeguardStop')


def _name(enum_value):
    return getattr(enum_value, 'name', str(enum_value))


class AuboError(RuntimeError):
    pass


class AuboClient:

    def __init__(self, ip, port=30004, user='aubo', password='123456', robot_name='', timeout_ms=3000):
        self.ip, self.port, self.user, self.password = ip, int(port), user, password
        self.robot_name = robot_name
        self.timeout_ms = timeout_ms
        self._lock = threading.RLock()
        self._rpc = None
        self._ok_codes = {0}

    # ── conexión ──
    @property
    def connected(self):
        return self._rpc is not None

    def connect(self):
        import pyaubo_sdk
        with self._lock:
            self.disconnect()
            rpc = pyaubo_sdk.RpcClient()
            rpc.setRequestTimeout(self.timeout_ms)
            rpc.connect(self.ip, self.port)
            if not rpc.hasConnected():
                raise AuboError(f'sin conexión RPC con {self.ip}:{self.port}')
            rpc.login(self.user, self.password)
            if not rpc.hasLogined():
                raise AuboError('login rechazado (revisa usuario y contraseña)')
            names = list(rpc.getRobotNames())
            name = self.robot_name or (names[0] if names else '')
            if not name:
                raise AuboError('el controlador no tiene robots configurados')
            robot = rpc.getRobotInterface(name)
            self._rpc, self.robot_name = rpc, name
            self._state = robot.getRobotState()
            self._motion = robot.getMotionControl()
            self._manage = robot.getRobotManage()
            self._config = robot.getRobotConfig()
            self._algo = robot.getRobotAlgorithm()
            try:
                self._ok_codes = {0, int(pyaubo_sdk.AuboErrorCodes.AUBO_INST_QUEUED)}
            except Exception:
                self._ok_codes = {0}
        return name

    def disconnect(self):
        with self._lock:
            if self._rpc is not None:
                try:
                    self._rpc.logout()
                    self._rpc.disconnect()
                except Exception:
                    pass
            self._rpc = None

    def _call(self, fn, *args):
        if self._rpc is None:
            raise AuboError('robot desconectado')
        with self._lock:
            return fn(*args)

    def ok(self, code):
        try:
            return int(code) in self._ok_codes
        except (TypeError, ValueError):
            return code is None or code is True

    # ── estado ──
    def read(self, slow=True):
        """Lectura del estado bajo el lock. Cada llamada RPC cuesta ~5 ms (VM por WiFi), así que
        las que cambian poco (modo, offset del TCP, freedrive) solo se leen si slow=True."""
        if self._rpc is None:
            raise AuboError('robot desconectado')
        with self._lock:
            s = self._state
            safety = _name(s.getSafetyModeType())
            out = {
                'joints': list(s.getJointPositions()),
                'tcp': list(s.getTcpPose()),
                'safety_mode': safety,
                'protective_stop': safety in PSTOP_MODES,
                'emergency_stop': safety in ESTOP_MODES,
                'steady': bool(s.isSteady()),
                'queue': int(self._motion.getQueueSize()),
                'exec_id': int(self._motion.getExecId()),
            }
            if slow:
                out['robot_mode'] = _name(s.getRobotModeType())
                out['tcp_offset'] = list(self._config.getTcpOffset())
                out['freedrive'] = bool(self._manage.isFreedriveEnabled())
            return out

    def motion_idle(self):
        with self._lock:
            return (self._motion.getQueueSize() == 0 and self._motion.getExecId() == -1
                    and self._state.isSteady())

    # ── movimiento (no bloqueante: el nodo vigila la llegada) ──
    def _move(self, fn, target, accel, speed):
        code = self._call(fn, list(target), float(accel), float(speed), 0.0, 0.0)
        for _ in range(3):
            if self.ok(code) or int(code) != REQUEST_IGNORE:
                break
            time.sleep(0.3)
            code = self._call(fn, list(target), float(accel), float(speed), 0.0, 0.0)
        if not self.ok(code):
            raise AuboError(f'el controlador rechazó el movimiento (código {int(code)})')

    def move_joint(self, joints, speed, accel):
        """joints en rad; speed en rad/s; accel en rad/s²."""
        self._move(self._motion.moveJoint, joints, accel, speed)

    def move_line(self, pose_rpy, speed, accel):
        """pose [x y z roll pitch yaw]; speed en m/s; accel en m/s²."""
        self._move(self._motion.moveLine, pose_rpy, accel, speed)

    def stop(self, decel=2.0):
        return self._call(self._motion.stopJoint, float(decel))

    # ── cinemática ──
    def fk(self, joints):
        pose, code = self._call(self._algo.forwardKinematics, list(joints))
        if not self.ok(code):
            raise AuboError(f'FK falló (código {int(code)})')
        return list(pose)

    def ik(self, seed, pose_rpy):
        q, code = self._call(self._algo.inverseKinematics, list(seed), list(pose_rpy))
        if not self.ok(code):
            raise AuboError(f'sin solución de IK (código {int(code)})')
        return list(q)

    def ik_all(self, pose_rpy):
        sols, code = self._call(self._algo.inverseKinematicsAll, list(pose_rpy))
        if not self.ok(code):
            raise AuboError(f'inverseKinematicsAll falló (código {int(code)})')
        return [list(s) for s in sols]

    # ── gestión ──
    def set_freedrive(self, enable, damping=0.6):
        if enable:
            self._call(self._config.setFreedriveDamp, [float(damping)] * 6)
        return self._call(self._manage.freedrive, bool(enable))

    def unlock_protective_stop(self):
        return self._call(self._manage.setUnlockProtectiveStop)


class MockAubo:
    """Robot simulado en memoria para probar sin controlador (robot:=mock).

    Solo mueve articulaciones (sin cinemática): FK, IK y moveLine responden con error.
    """

    def __init__(self, joints=None):
        self.robot_name = 'mock'
        self._joints = list(joints or [0.0] * 6)
        self._target = list(self._joints)
        self._speed = 0.5
        self._last = time.monotonic()
        self._lock = threading.Lock()

    connected = True

    def connect(self):
        return 'mock'

    def disconnect(self):
        pass

    def ok(self, code):
        return code == 0

    def _step(self):
        now = time.monotonic()
        dt, self._last = now - self._last, now
        for i in range(6):
            err = self._target[i] - self._joints[i]
            step = self._speed * dt
            self._joints[i] = self._target[i] if abs(err) <= step else self._joints[i] + step * (1 if err > 0 else -1)

    def read(self, slow=True):
        with self._lock:
            self._step()
            moving = any(abs(t - j) > 1e-6 for t, j in zip(self._target, self._joints))
            return {'joints': list(self._joints), 'tcp': [0.0] * 6, 'tcp_offset': [0.0] * 6,
                    'robot_mode': 'Running', 'safety_mode': 'Normal', 'protective_stop': False,
                    'emergency_stop': False, 'steady': not moving, 'queue': 0,
                    'exec_id': 1 if moving else -1, 'freedrive': False}

    def motion_idle(self):
        return self.read()['steady']

    def move_joint(self, joints, speed, accel):
        with self._lock:
            self._step()
            self._target, self._speed = list(joints), max(0.05, float(speed))

    def stop(self, decel=2.0):
        with self._lock:
            self._step()
            self._target = list(self._joints)
        return 0

    def move_line(self, pose_rpy, speed, accel):
        raise AuboError('moveLine no disponible en el robot simulado')

    def fk(self, joints):
        raise AuboError('FK no disponible en el robot simulado')

    def ik(self, seed, pose_rpy):
        raise AuboError('IK no disponible en el robot simulado')

    def ik_all(self, pose_rpy):
        raise AuboError('IK no disponible en el robot simulado')

    def set_freedrive(self, enable, damping=0.6):
        return 0

    def unlock_protective_stop(self):
        return 0
