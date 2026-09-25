"""system_monitor — checks de arranque, warm-up y salud del sistema.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 1.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.msg import ArmState, CameraStatus, GripperState, HealthItem, SystemHealth

from atole_common.config_view import ConfigView
from atole_common.qos import LATCHED

PHASE = 1


class SystemMonitor(Node):

    def __init__(self):
        super().__init__('system_monitor')
        self.health_pub = self.create_publisher(SystemHealth, '/atole/system/health', LATCHED)
        self.seen = {}
        self.config = ConfigView(self)
        self.create_subscription(ArmState, '/atole/arm/state', lambda m: self._mark('robot', m), LATCHED)
        self.create_subscription(GripperState, '/atole/gripper/state', lambda m: self._mark('gripper', m), LATCHED)
        self.create_subscription(CameraStatus, '/atole/cameras/status', lambda m: self._mark('cameras', m), LATCHED)
        # Fase 0: solo informa qué nodos publican; los checks reales llegan en la Fase 1.
        self.create_timer(1.0, self._publish)
        self.get_logger().info('system_monitor listo (esqueleto; lógica en la Fase 1)')

    def _mark(self, name, msg):
        self.seen[name] = msg

    def _publish(self):
        items = [HealthItem(name='config',
                            level=HealthItem.OK if self.config.ready else HealthItem.ERROR,
                            message=f'versión {self.config.version}' if self.config.ready else 'sin /atole/config')]
        for name in ('robot', 'gripper', 'cameras'):
            items.append(HealthItem(name=name, level=HealthItem.UNKNOWN,
                                    message='publica (sin checks todavía)' if name in self.seen else 'sin datos'))
        msg = SystemHealth(phase='INIT_CHECKS', ready=False, items=items)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.health_pub.publish(msg)


def main(args=None):
    run_node(SystemMonitor, args)


if __name__ == '__main__':
    main()
