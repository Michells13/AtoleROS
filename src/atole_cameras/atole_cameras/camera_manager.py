"""camera_manager — EtH activa (cam0 | cam1 | camC) + cam2; swap y display.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 2.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.action import SetCameraMode
from atole_interfaces.msg import CameraSlot, CameraStatus
from atole_interfaces.srv import SelectEth, SetDisplay

from atole_common.config_view import ConfigView
from atole_common.qos import LATCHED

CAMERAS = ('cam0', 'cam1', 'camC', 'cam2')

PHASE = 2


class CameraManager(Node):

    def __init__(self):
        super().__init__('camera_manager')
        self.status_pub = self.create_publisher(CameraStatus, '/atole/cameras/status', LATCHED)
        stub_action(self, SetCameraMode, '/atole/cameras/set_mode', PHASE)
        stub_service(self, SelectEth, '/atole/cameras/select_eth', PHASE)
        stub_service(self, SetDisplay, '/atole/cameras/set_display', PHASE)
        # Ya en Fase 0: publica las cámaras declaradas en Config.xml (sin detectar hardware).
        self.config = ConfigView(self, on_update=self._publish_status)
        self.get_logger().info('camera_manager listo (esqueleto; lógica en la Fase 2)')

    def _publish_status(self, cfg):
        msg = CameraStatus(source='live', active_eth=cfg.get('Cameras/SelectedEtH', ''))
        msg.header.stamp = self.get_clock().now().to_msg()
        for cam in CAMERAS:
            c = cfg.section(f'Cameras/{cam}')
            calib = cfg.section(f'Calibrations/{cam}')
            msg.cameras.append(CameraSlot(
                id=cam, model=c.get('Model', ''), serial=c.get('Serial', ''), role=c.get('Role', ''),
                position=c.get('Position', ''), display=c.get('Display', 'true') == 'true',
                calibrated=bool(calib.get('Matrix')) and calib.get('Serial') == c.get('Serial')))
        self.status_pub.publish(msg)


def main(args=None):
    run_node(CameraManager, args)


if __name__ == '__main__':
    main()
