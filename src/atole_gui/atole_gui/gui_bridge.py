"""gui_bridge — reduce datos para la web GUI (overlays, resumen de pods).

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 6.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from std_msgs.msg import String

PHASE = 6


class GuiBridge(Node):

    def __init__(self):
        super().__init__('gui_bridge')
        self.summary_pub = self.create_publisher(String, '/atole/gui/summary', 10)
        self.get_logger().info('gui_bridge listo (esqueleto; lógica en la Fase 6)')


def main(args=None):
    run_node(GuiBridge, args)


if __name__ == '__main__':
    main()
