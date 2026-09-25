"""grip_twist_controller — control del gripper RGI-100 (grip + twist).

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 1.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.action import GripperCommand
from atole_interfaces.msg import GripperState
from atole_interfaces.srv import ScanPort
from std_srvs.srv import Trigger

from atole_common.qos import LATCHED

PHASE = 1


class GripTwistController(Node):

    def __init__(self):
        super().__init__('grip_twist_controller')
        self.state_pub = self.create_publisher(GripperState, '/atole/gripper/state', LATCHED)
        stub_action(self, GripperCommand, '/atole/gripper/command', PHASE)
        stub_service(self, Trigger, '/atole/gripper/reinit', PHASE)
        stub_service(self, Trigger, '/atole/gripper/reconnect', PHASE)
        stub_service(self, ScanPort, '/atole/gripper/scan_port', PHASE)
        self.get_logger().info('grip_twist_controller listo (esqueleto; lógica en la Fase 1)')


def main(args=None):
    run_node(GripTwistController, args)


if __name__ == '__main__':
    main()
