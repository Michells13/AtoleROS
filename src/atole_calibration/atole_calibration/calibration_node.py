"""calibration_node — calibración EtH/EiH (backend PozoleV3; Tecnalia en breve).

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 7.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.action import RunCalibSequence
from atole_interfaces.srv import CaptureCalibPose, ComputeCalibration, DetectBoard, SaveCalibration

PHASE = 7


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')
        stub_service(self, DetectBoard, '/atole/calibration/detect_board', PHASE)
        stub_service(self, CaptureCalibPose, '/atole/calibration/capture_pose', PHASE)
        stub_service(self, ComputeCalibration, '/atole/calibration/compute', PHASE)
        stub_service(self, SaveCalibration, '/atole/calibration/save', PHASE)
        stub_action(self, RunCalibSequence, '/atole/calibration/run_sequence', PHASE)
        self.get_logger().info('calibration_node listo (esqueleto; lógica en la Fase 7)')


def main(args=None):
    run_node(CalibrationNode, args)


if __name__ == '__main__':
    main()
