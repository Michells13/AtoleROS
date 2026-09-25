"""detector_node — Mask R-CNN (modelo seleccionable en Config.xml).

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 3.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.msg import DetectionArray
from atole_interfaces.srv import DetectOnce
from std_srvs.srv import SetBool, Trigger

PHASE = 3


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        self.det_pub = self.create_publisher(DetectionArray, '/atole/perception/detections', 10)
        stub_service(self, DetectOnce, '/atole/perception/detect_once', PHASE)
        stub_service(self, SetBool, '/atole/perception/set_stream', PHASE)
        stub_service(self, Trigger, '/atole/perception/detector/warmup', PHASE)
        self.get_logger().info('detector_node listo (esqueleto; lógica en la Fase 3)')


def main(args=None):
    run_node(DetectorNode, args)


if __name__ == '__main__':
    main()
