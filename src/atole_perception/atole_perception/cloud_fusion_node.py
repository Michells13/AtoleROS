"""cloud_fusion_node — fusión de nubes: EiH 2 vistas y EtH + EiH.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 5.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.srv import FuseViews

PHASE = 5


class CloudFusionNode(Node):

    def __init__(self):
        super().__init__('cloud_fusion_node')
        stub_service(self, FuseViews, '/atole/perception/fuse_views', PHASE)
        self.get_logger().info('cloud_fusion_node listo (esqueleto; lógica en la Fase 5)')


def main(args=None):
    run_node(CloudFusionNode, args)


if __name__ == '__main__':
    main()
