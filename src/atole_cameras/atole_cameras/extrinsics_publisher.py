"""extrinsics_publisher — publica en /tf_static el montaje de cada cámara desde Config.xml.

Config.xml guarda, por cámara, la matriz parent ← marco óptico (convención de PozoleV3; el marco
óptico de la ZED es X derecha, Y abajo, Z adelante, igual que el óptico de ROS).

- Cámaras reales: la ZED publica su propia cadena <cam>_camera_link → … → <cam>_left_camera_frame_optical,
  así que aquí se publica parent → <cam>_camera_link = T(parent←óptico) · T(link←óptico)⁻¹. El resultado
  es que base_link → óptico coincide exactamente con la matriz calibrada.
- Modo SIM: no hay cadena de la ZED; se publica parent → <cam>_left_camera_frame_optical directamente.
- Solo se publica la calibración si su serial coincide con el de la cámara en Config.xml.

Además se publica parent → <cam>_calibrated_optical = matriz calibrada, exacta y con un único
camino. Es el marco que usa la percepción: la ZED publica camera_center → left_camera_frame dos
veces (nominal en /tf_static desde su URDF, calibrado en /tf desde el nodo; en cam1 difieren
0.38 mm), así que base_link → <cam>_left_camera_frame_optical no es exacto ni determinista.
Por eso esa cadena solo se usa para visualizar.
"""
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from atole_common.config_view import ConfigView
from atole_common.stubs import run_node

CAMERAS = ('cam0', 'cam1', 'camC', 'cam2')


def to_matrix(transform):
    t, q = transform.translation, transform.rotation
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def to_transform(m, parent, child, stamp):
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in m[:3, 3])
    x, y, z, w = Rotation.from_matrix(m[:3, :3]).as_quat()
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = x, y, z, w
    return t


class ExtrinsicsPublisher(Node):

    def __init__(self):
        super().__init__('extrinsics_publisher')
        self.sim = self.declare_parameter('sim', False).value
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.published = {}          # cam -> matriz publicada
        self.warned = set()
        self.config = ConfigView(self, on_update=lambda _c: self.published.clear())
        self.create_timer(1.0, self._update)
        self.get_logger().info(f'extrinsics_publisher arrancado ({"SIM: parent→óptico directo" if self.sim else "parent→camera_link"})')

    def _calibration(self, cam):
        c = self.config.section(f'Calibrations/{cam}')
        serial = self.config.get(f'Cameras/{cam}/Serial', '')
        values = c.get('Matrix', '').split()
        if len(values) != 16:
            return None, None, 'sin matriz'
        if not serial or c.get('Serial') != serial:
            return None, None, f'serial de la calibración ({c.get("Serial")}) ≠ serial de la cámara ({serial or "sin leer"})'
        return np.array([float(v) for v in values]).reshape(4, 4), c.get('Parent', 'base_link'), None

    def _update(self):
        if not self.config.ready:
            return
        stamp = self.get_clock().now().to_msg()
        for cam in CAMERAS:
            matrix, parent, problem = self._calibration(cam)
            if problem:
                if cam not in self.warned:
                    self.get_logger().warn(f'{cam}: no se publica su TF ({problem})')
                    self.warned.add(cam)
                continue
            optical = f'{cam}_left_camera_frame_optical'
            calibrated = f'{cam}_calibrated_optical'
            if cam not in self.published:
                self.broadcaster.sendTransform(to_transform(matrix, parent, calibrated, stamp))
            if self.sim:
                target, child = matrix, optical
            else:
                link = f'{cam}_camera_link'
                try:
                    link_optical = to_matrix(self.tf_buffer.lookup_transform(link, optical, Time()).transform)
                except Exception:
                    continue           # la cámara aún no publica su cadena (no está activa)
                target, child = matrix @ np.linalg.inv(link_optical), link
            previous = self.published.get(cam)
            if previous is not None and np.allclose(previous, target, atol=1e-3):   # la cadena ZED oscila <0.4 mm
                continue
            self.broadcaster.sendTransform(to_transform(target, parent, child, stamp))
            self.published[cam] = target
            self.warned.discard(cam)
            self.get_logger().info(f'{cam}: TF {parent} → {child} publicada (t = {np.round(matrix[:3, 3], 4).tolist()})')


def main(args=None):
    run_node(ExtrinsicsPublisher, args)


if __name__ == '__main__':
    main()
