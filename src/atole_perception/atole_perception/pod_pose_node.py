"""pod_pose_node — filtro de workspace + completion + superquadric + PCA.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 3.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.action import EstimatePods
from atole_interfaces.msg import PodArray
from std_srvs.srv import Trigger

PHASE = 3


class PodPoseNode(Node):

    def __init__(self):
        super().__init__('pod_pose_node')
        self.pods_pub = self.create_publisher(PodArray, '/atole/perception/pods', 10)
        stub_action(self, EstimatePods, '/atole/perception/estimate_pods', PHASE)
        stub_service(self, Trigger, '/atole/perception/pod_pose/warmup', PHASE)
        self.get_logger().info('pod_pose_node listo (esqueleto; lógica en la Fase 3)')


def main(args=None):
    run_node(PodPoseNode, args)


if __name__ == '__main__':
    main()
