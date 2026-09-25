"""arm_driver — driver del AUBO iS10, único nodo que habla con el robot.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 1.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.action import MoveJoints, MovePose
from atole_interfaces.msg import ArmState
from atole_interfaces.srv import AcquireControl, ComputeFk, ComputeIk, JogJoint, SetFreedrive
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from atole_common.qos import LATCHED

JOINT_NAMES = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
               'wrist1_joint', 'wrist2_joint', 'wrist3_joint']

PHASE = 1


class ArmDriver(Node):

    def __init__(self):
        super().__init__('arm_driver')
        self.backend = self.declare_parameter('backend', 'mock').value   # mock | vm | real
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_pub = self.create_publisher(ArmState, '/atole/arm/state', LATCHED)
        self.fault_pub = self.create_publisher(ArmState, '/atole/arm/fault', LATCHED)
        stub_action(self, MoveJoints, '/atole/arm/move_joints', PHASE)
        stub_action(self, MovePose, '/atole/arm/move_pose', PHASE)
        for name in ('stop', 'unlock_pstop', 'reconnect'):
            stub_service(self, Trigger, f'/atole/arm/{name}', PHASE)
        stub_service(self, SetBool, '/atole/arm/set_j6_lock', PHASE)
        stub_service(self, SetFreedrive, '/atole/arm/set_freedrive', PHASE)
        stub_service(self, ComputeIk, '/atole/arm/compute_ik', PHASE)
        stub_service(self, ComputeFk, '/atole/arm/compute_fk', PHASE)
        stub_service(self, JogJoint, '/atole/arm/jog_joint', PHASE)
        stub_service(self, AcquireControl, '/atole/arm/acquire_control', PHASE)
        if self.backend == 'mock':
            # Solo en mock: articulaciones a cero para que robot_state_publisher dibuje el brazo.
            self.create_timer(0.1, self._publish_mock)
        self.get_logger().info('arm_driver listo (esqueleto; lógica en la Fase 1)')

    def _publish_mock(self):
        now = self.get_clock().now().to_msg()
        self.joint_pub.publish(JointState(header=self._header(now), name=JOINT_NAMES, position=[0.0] * 6))
        state = ArmState(connected=False, robot_mode='MOCK', last_fault='')
        state.header = self._header(now)
        self.state_pub.publish(state)

    @staticmethod
    def _header(stamp):
        from std_msgs.msg import Header
        return Header(stamp=stamp, frame_id='base_link')


def main(args=None):
    run_node(ArmDriver, args)


if __name__ == '__main__':
    main()
