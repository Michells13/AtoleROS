"""extrinsics_publisher — publica la TF de montaje de cada cámara desde Config.xml.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 2.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_common.config_view import ConfigView

PHASE = 2


class ExtrinsicsPublisher(Node):

    def __init__(self):
        super().__init__('extrinsics_publisher')
        self.config = ConfigView(self)
        self.get_logger().info('extrinsics_publisher listo (esqueleto; lógica en la Fase 2)')


def main(args=None):
    run_node(ExtrinsicsPublisher, args)


if __name__ == '__main__':
    main()
