"""system_monitor — checks de arranque y salud continua del sistema.

Publica /atole/system/health (latched) cada segundo.
Robot, gripper, calibraciones, cámaras (streaming por heartbeat; en SIM, las del dataset) y warm-up
de percepción: al arrancar (Perception/WarmupOnStart) llama a los warm-up de detector_node
(Mask R-CNN EtH y EiH) y pod_pose_node (workers AdaPoinTr EtH y EiH). Hasta que terminan, el
sistema no pasa a READY.
"""
from atole_interfaces.msg import ArmState, CameraStatus, GripperState, HealthItem, SystemHealth
from rclpy.node import Node
from std_srvs.srv import Trigger

from atole_common.config_view import ConfigView
from atole_common.qos import LATCHED
from atole_common.stubs import run_node

OK, WARN, ERROR, UNKNOWN = HealthItem.OK, HealthItem.WARN, HealthItem.ERROR, HealthItem.UNKNOWN
WARMUPS = {'detector': '/atole/perception/detector/warmup', 'pod_pose': '/atole/perception/pod_pose/warmup'}


class SystemMonitor(Node):

    def __init__(self):
        super().__init__('system_monitor')
        self.arm = self.gripper = self.cameras = None
        self._last_levels = {}
        self.health_pub = self.create_publisher(SystemHealth, '/atole/system/health', LATCHED)
        self.config = ConfigView(self)
        self.create_subscription(ArmState, '/atole/arm/state', lambda m: setattr(self, 'arm', m), LATCHED)
        self.create_subscription(GripperState, '/atole/gripper/state', lambda m: setattr(self, 'gripper', m), LATCHED)
        self.create_subscription(CameraStatus, '/atole/cameras/status', lambda m: setattr(self, 'cameras', m), LATCHED)
        self.warmup = {}         # nombre -> None (en curso) | (ok, mensaje)
        self.warmup_clients = {name: self.create_client(Trigger, srv) for name, srv in WARMUPS.items()}
        self.create_timer(1.0, self._publish)
        self.get_logger().info('system_monitor listo')

    def _robot(self):
        a = self.arm
        if a is None:
            return ERROR, 'arm_driver no publica estado'
        if not a.connected:
            return ERROR, 'robot desconectado'
        if a.emergency_stop or a.protective_stop:
            return ERROR, f'robot en {a.safety_mode}'
        if a.robot_mode not in ('Running', 'MOCK'):
            return WARN, f'robot en modo {a.robot_mode}'
        return OK, f'{a.robot_mode} · {a.safety_mode or "Normal"}'

    def _gripper(self):
        g = self.gripper
        if g is None:
            return ERROR, 'grip_twist_controller no publica estado'
        if not g.connected:
            return ERROR, 'gripper desconectado'
        if g.init_state != 1:
            return WARN, f'gripper {g.init_state_name or "sin inicializar"} (llama a /atole/gripper/reinit)'
        return OK, f'{g.port} · apertura {g.last_opening} · {g.last_angle_deg}°'

    def _cameras(self):
        c = self.cameras
        if c is None:
            return [HealthItem(name='cameras', level=ERROR, message='camera_manager no publica estado')]
        slots = {s.id: s for s in c.cameras}
        items = []
        for cam in (c.active_eth, 'cam2'):
            s = slots.get(cam)
            if s is None:
                items.append(HealthItem(name=f'cam_{cam or "eth"}', level=ERROR, message='cámara no configurada'))
            elif s.streaming:
                items.append(HealthItem(name=f'cam_{cam}', level=OK, message=f'transmitiendo ({c.source})'))
            elif c.source == 'sim':
                # En SIM solo se reproducen cam0/cam1: cam2 no bloquea (sin refinamiento EiH).
                items.append(HealthItem(name=f'cam_{cam}', level=UNKNOWN, message='SIM: solo cam0/cam1 (no hay dataset EiH con pose)')
                             if cam == 'cam2' else
                             HealthItem(name=f'cam_{cam}', level=WARN, message='SIM: carga un dataset (/atole/sim/load)'))
            elif c.swap_in_progress or s.active:
                items.append(HealthItem(name=f'cam_{cam}', level=WARN, message='arrancando'))
            else:
                items.append(HealthItem(name=f'cam_{cam}', level=ERROR,
                                        message='no conectada' if c.source == 'live' and not s.connected else 'sin datos'))
        return items

    def _calibrations(self):
        items = []
        selected = self.config.get('Cameras/SelectedEtH', '')
        for cam in (selected, 'cam2'):
            if not cam:
                items.append(HealthItem(name='calib_eth', level=ERROR, message='no hay cámara EtH seleccionada'))
                continue
            serial = self.config.get(f'Cameras/{cam}/Serial', '')
            calib = self.config.section(f'Calibrations/{cam}')
            if not calib.get('Matrix'):
                level, text = ERROR, 'sin matriz de calibración'
            elif not serial:
                level, text = WARN, 'serial de la cámara sin leer: no se puede confirmar la calibración'
            elif calib.get('Serial') != serial:
                level, text = ERROR, f'la calibración es de otra cámara (serial {calib.get("Serial")} ≠ {serial})'
            else:
                level, text = OK, f'serial {serial} · {calib.get("Backend", "")}'
            items.append(HealthItem(name=f'calib_{cam}', level=level, message=text))
        return items

    def _warmup(self):
        if not self.config.ready:
            return HealthItem(name='warmup', level=WARN, message='esperando configuración')
        if not self.config.get_bool('Perception/WarmupOnStart', True):
            return HealthItem(name='warmup', level=UNKNOWN, message='desactivado (Perception/WarmupOnStart)')
        for name, client in self.warmup_clients.items():      # se lanza una sola vez, cuando el servicio aparece
            if name not in self.warmup and client.service_is_ready():
                self.warmup[name] = None
                client.call_async(Trigger.Request()).add_done_callback(lambda f, n=name: self._warmup_done(n, f))
        pending = [n for n in WARMUPS if self.warmup.get(n) is None]
        failed = [r[1] for r in self.warmup.values() if r and not r[0]]
        if failed:
            return HealthItem(name='warmup', level=ERROR, message=' · '.join(failed))
        if pending:
            waiting = [n for n in pending if n not in self.warmup]
            return HealthItem(name='warmup', level=WARN, message='calentando: ' + ', '.join(pending)
                              + (f' (esperando servicio de {", ".join(waiting)})' if waiting else ''))
        return HealthItem(name='warmup', level=OK, message=' · '.join(r[1] for r in self.warmup.values()))

    def _warmup_done(self, name, future):
        try:
            res = future.result()
            self.warmup[name] = (res.success, res.message)
        except Exception as e:
            self.warmup[name] = (False, f'{name}: {e}')

    def _publish(self):
        items = [HealthItem(name='config', level=OK if self.config.ready else ERROR,
                            message=f'versión {self.config.version}' if self.config.ready else 'sin /atole/config')]
        level, text = self._robot()
        items.append(HealthItem(name='robot', level=level, message=text))
        level, text = self._gripper()
        items.append(HealthItem(name='gripper', level=level, message=text))
        if self.config.ready:
            items += self._calibrations()
        items += self._cameras()
        items.append(self._warmup())

        checked = [i for i in items if i.level != UNKNOWN]
        errors = [i for i in checked if i.level == ERROR]
        ready = self.config.ready and not errors and all(i.level == OK for i in checked)
        msg = SystemHealth(phase='READY' if ready else ('FAULT' if errors else 'INIT_CHECKS'), ready=ready, items=items)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.health_pub.publish(msg)
        for i in items:     # registra solo los cambios de nivel
            if self._last_levels.get(i.name) != i.level:
                text = f'{i.name}: {["OK", "AVISO", "ERROR", "?"][i.level]} — {i.message}'
                # rclpy no deja cambiar la severidad desde una misma línea: una llamada por nivel.
                if i.level in (OK, UNKNOWN):
                    self.get_logger().info(text)
                else:
                    self.get_logger().warn(text)
                self._last_levels[i.name] = i.level


def main(args=None):
    run_node(SystemMonitor, args)


if __name__ == '__main__':
    main()
