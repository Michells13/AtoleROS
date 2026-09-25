"""gui_bridge — reduce los datos de las cámaras para la web GUI y Lichtblick.

Solo trabaja con las cámaras que transmiten y tienen activada su visualización (CameraSlot.display /
.cloud, /atole/cameras/set_display):
- /atole/gui/<cam>/image/compressed   miniatura JPEG (Gui/ThumbWidth px) a Gui/ThumbRateHz.
- /atole/gui/<cam>/cloud              nube de escena submuestreada (Gui/CloudStride) con color, en
                                      <cam>_calibrated_optical (la calibración exacta), a Gui/CloudRateHz.
- /atole/gui/detections/compressed    última detección de Mask R-CNN dibujada sobre la miniatura.
Las imágenes de la ZED pesan ~12 MB: en vez de suscribirse de forma permanente (Python recibiría
todos los frames a la tasa de la cámara) se usan suscripciones de instantánea: se crean, reciben un
frame y se destruyen.
"""
import time

import cv2
import numpy as np
from atole_interfaces.msg import CameraStatus, DetectionArray
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField

from atole_common.config_view import ConfigView
from atole_common.qos import LATCHED
from atole_common.stubs import run_node
from atole_perception.mask_cloud import full_mask
from atole_perception.ros_utils import image_to_array, to_bgr

IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
COLORS = [(0, 170, 255), (80, 220, 80), (255, 120, 60), (220, 80, 220), (60, 220, 220), (255, 200, 0)]


class Snapshot:
    """Suscripción que vive solo hasta recibir un mensaje."""

    def __init__(self, node, msg_type, topic):
        self.node, self.msg_type, self.topic = node, msg_type, topic
        self.sub, self.msg, self.requested = None, None, 0.0

    def request(self):
        if self.sub is None:
            self.msg, self.requested = None, time.monotonic()
            self.sub = self.node.create_subscription(self.msg_type, self.topic, self._on_msg, IMAGE_QOS,
                                                     event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))

    def _on_msg(self, msg):
        self.msg = msg

    def collect(self, timeout=5.0):
        """Devuelve el mensaje si llegó (y cierra la suscripción) o None."""
        if self.sub is None:
            return None
        if self.msg is None and time.monotonic() - self.requested < timeout:
            return None
        self.node.destroy_subscription(self.sub)
        self.sub, msg, self.msg = None, self.msg, None
        return msg


def jpeg(bgr, stamp, frame_id=''):
    msg = CompressedImage(format='jpeg')
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    msg.data = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
    return msg


def colored_cloud(xyz, bgr, frame_id, stamp):
    """PointCloud2 x, y, z, rgb (float32 con el color empaquetado, como lo lee Lichtblick)."""
    rgb = (bgr[:, 2].astype(np.uint32) << 16) | (bgr[:, 1].astype(np.uint32) << 8) | bgr[:, 0].astype(np.uint32)
    data = np.empty(len(xyz), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
    data['x'], data['y'], data['z'], data['rgb'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], rgb
    msg = PointCloud2(height=1, width=len(data), is_bigendian=False, is_dense=True, point_step=16, row_step=16 * len(data))
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    msg.fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
    msg.data = data.tobytes()
    msg.header.frame_id, msg.header.stamp = frame_id, stamp
    return msg


class GuiBridge(Node):

    def __init__(self):
        super().__init__('gui_bridge')
        self.config = ConfigView(self)
        self.status = None
        self.snaps = {}           # (cam, kind) -> Snapshot
        self.thumbs = {}          # cam -> (bgr miniatura, escala, stamp)
        self.pubs = {}
        self.last = {}            # (cam, kind) -> monotonic del último envío
        self.depth, self.info = {}, {}   # cam -> último mensaje recibido (nube)
        self.create_subscription(CameraStatus, '/atole/cameras/status', lambda m: setattr(self, 'status', m), LATCHED)
        self.create_subscription(DetectionArray, '/atole/perception/detections', self._on_detections, 10)
        self.overlay_pub = self.create_publisher(CompressedImage, '/atole/gui/detections/compressed', 2)
        self.create_timer(0.1, self._tick)
        self.get_logger().info('gui_bridge listo')

    def _pub(self, cam, kind):
        key = (cam, kind)
        if key not in self.pubs:
            topic, t = {'image': (f'/atole/gui/{cam}/image/compressed', CompressedImage),
                        'cloud': (f'/atole/gui/{cam}/cloud', PointCloud2)}[kind]
            self.pubs[key] = self.create_publisher(t, topic, 2)
        return self.pubs[key]

    def _snap(self, cam, kind):
        key = (cam, kind)
        if key not in self.snaps:
            base = f'/atole/zed/{cam}'
            self.snaps[key] = {'rgb': lambda: Snapshot(self, Image, f'{base}/rgb/color/rect/image'),
                               'depth': lambda: Snapshot(self, Image, f'{base}/depth/depth_registered'),
                               'info': lambda: Snapshot(self, CameraInfo, f'{base}/rgb/color/rect/camera_info')}[kind]()
        return self.snaps[key]

    def _due(self, cam, kind, rate_hz):
        return time.monotonic() - self.last.get((cam, kind), 0.0) >= 1.0 / max(rate_hz, 0.05)

    def _tick(self):
        if self.status is None or not self.config.ready:
            return
        g = self.config.section('Gui')
        thumb_rate, cloud_rate = float(g.get('ThumbRateHz', 2.0)), float(g.get('CloudRateHz', 1.0))
        for slot in self.status.cameras:
            cam = slot.id
            if not slot.streaming or not (slot.display or slot.cloud):
                continue
            rate = max(thumb_rate if slot.display else 0.0, cloud_rate if slot.cloud else 0.0)
            rgb = self._snap(cam, 'rgb')
            msg = rgb.collect()
            if msg is not None:
                self._on_rgb(cam, msg, publish=slot.display)
            elif rgb.sub is None and self._due(cam, 'rgb', rate):
                self.last[(cam, 'rgb')] = time.monotonic()
                rgb.request()
            if slot.cloud:
                self._cloud_step(cam, cloud_rate)

    def _on_rgb(self, cam, msg, publish):
        try:
            bgr = to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(f'{cam}: {e}', throttle_duration_sec=30.0)
            return
        width = int(float(self.config.get('Gui/ThumbWidth', 640)))
        scale = width / bgr.shape[1]
        small = cv2.resize(bgr, (width, int(round(bgr.shape[0] * scale))), interpolation=cv2.INTER_AREA)
        self.thumbs[cam] = (small, scale, msg.header.stamp)
        if publish:
            self._pub(cam, 'image').publish(jpeg(small, msg.header.stamp, msg.header.frame_id))

    def _cloud_step(self, cam, rate):
        depth, info = self._snap(cam, 'depth'), self._snap(cam, 'info')
        for snap, store in ((depth, self.depth), (info, self.info)):
            msg = snap.collect()
            if msg is not None:
                store[cam] = msg
        if cam in self.depth and cam in self.info:
            self._publish_cloud(cam, self.depth.pop(cam), self.info.pop(cam))
        if depth.sub is None and info.sub is None and self._due(cam, 'cloud', rate):
            self.last[(cam, 'cloud')] = time.monotonic()
            depth.request()
            info.request()

    def _publish_cloud(self, cam, depth_msg, info):
        stride = max(1, int(float(self.config.get('Gui/CloudStride', 6))))
        z = image_to_array(depth_msg)[::stride, ::stride].astype(np.float32)
        h, w = z.shape
        v, u = np.mgrid[0:h, 0:w] * stride
        k = info.k
        ok = np.isfinite(z) & (z > 0.05) & (z < 20.0)
        x = (u - k[2]) * z / k[0]
        y = (v - k[5]) * z / k[4]
        xyz = np.stack([x[ok], y[ok], z[ok]], axis=1)
        thumb = self.thumbs.get(cam)
        if thumb is not None:
            small, scale, _ = thumb
            uu = np.clip((u[ok] * scale).astype(int), 0, small.shape[1] - 1)
            vv = np.clip((v[ok] * scale).astype(int), 0, small.shape[0] - 1)
            colors = small[vv, uu]
        else:
            colors = np.full((len(xyz), 3), 180, np.uint8)
        self._pub(cam, 'cloud').publish(colored_cloud(xyz, colors, f'{cam}_calibrated_optical', depth_msg.header.stamp))

    def _on_detections(self, msg):
        thumb = self.thumbs.get(msg.camera)
        width = int(float(self.config.get('Gui/ThumbWidth', 640)))
        if thumb is not None:
            canvas, scale = thumb[0].copy(), thumb[1]
        else:                     # sin imagen reciente de esa cámara: solo las máscaras
            scale = width / max(1, msg.image_width)
            canvas = np.full((int(msg.image_height * scale), width, 3), 40, np.uint8)
        h, w = canvas.shape[:2]
        for det in msg.detections:
            color = COLORS[det.id % len(COLORS)]
            if det.mask.data:
                m = full_mask(image_to_array(det.mask), det.mask_xy, msg.image_height, msg.image_width)
                m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST) > 0
                canvas[m] = (0.5 * canvas[m] + 0.5 * np.array(color)).astype(np.uint8)
            x, y, bw, bh = (int(round(v * scale)) for v in det.bbox_xywh)
            cv2.rectangle(canvas, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(canvas, f'{det.id} {det.class_name} {det.score:.2f}', (x, max(14, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        cv2.putText(canvas, f'{msg.camera} · {len(msg.detections)} detecciones · {msg.inference_ms:.0f} ms', (8, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        self.overlay_pub.publish(jpeg(canvas, msg.header.stamp, msg.header.frame_id))


def main(args=None):
    run_node(GuiBridge, args, single_threaded=True)     # crea y destruye suscripciones (Snapshot)


if __name__ == '__main__':
    main()
