"""mission_manager — máquina de estados de la cosecha (todos | siguiente | paso a paso).

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 4.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.action import Harvest
from atole_interfaces.msg import MissionStatus
from std_srvs.srv import SetBool, Trigger

from atole_common.qos import LATCHED

PHASE = 4


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        self.status_pub = self.create_publisher(MissionStatus, '/atole/mission/status', LATCHED)
        stub_action(self, Harvest, '/atole/mission/harvest', PHASE)
        stub_service(self, Trigger, '/atole/mission/confirm_step', PHASE)
        stub_service(self, Trigger, '/atole/mission/abort', PHASE)
        stub_service(self, SetBool, '/atole/mission/set_step_mode', PHASE)
        status = MissionStatus(state='IDLE', message='esqueleto: misión no implementada')
        status.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(status)
        self.get_logger().info('mission_manager listo (esqueleto; lógica en la Fase 4)')


def main(args=None):
    run_node(MissionManager, args)


if __name__ == '__main__':
    main()
