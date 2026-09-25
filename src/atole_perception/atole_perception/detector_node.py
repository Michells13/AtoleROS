"""detector_node — Mask R-CNN sobre las cámaras (modelo y umbrales en Config.xml).

- /atole/perception/detect_once (DetectOnce): detecta sobre la imagen que viene en la petición
  (pod_pose_node manda el frame que congeló) o sobre el último frame de la cámara.
  cam2 usa el modelo EiH (Perception/EihModelPath); el resto, Perception/ModelPath.
- /atole/perception/set_stream (SetBool): detección continua sobre la EtH seleccionada a
  Perception/StreamRateHz; publica /atole/perception/detections (con máscaras) para la GUI.
- /atole/perception/detector/warmup (Trigger): carga los dos modelos y hace una inferencia ficticia.
Los modelos se cargan una vez y se recargan solos si cambia su ruta, perfil o umbral en Config.xml.
"""
import threading
import time

import numpy as np
from atole_interfaces.msg import Detection, DetectionArray
from atole_interfaces.srv import DetectOnce
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

from atole_common.config_view import ConfigView
from atole_common.stubs import run_node
from atole_perception.maskrcnn import MaskRCNN
from atole_perception.ros_utils import mono8, to_bgr

FRAME_TIMEOUT_S = 5.0
# RELIABLE como publican la ZED y dataset_player: en best-effort Fast DDS pierde casi todos los frames de
# 12 MB (medido: 0.9 Hz frente a 6 Hz). El perfil no bloqueante evita que un lector lento frene a la cámara.
IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        self.models = {}                 # 'eth' | 'eih' -> MaskRCNN
        self.latest = {}                 # cámara -> último Image
        self.frame_event = {}
        self.image_subs = {}
        self.infer_lock = threading.Lock()
        self.stream_timer = None
        group = ReentrantCallbackGroup()
        self.config = ConfigView(self)
        self.det_pub = self.create_publisher(DetectionArray, '/atole/perception/detections', 10)
        self.create_service(DetectOnce, '/atole/perception/detect_once', self._srv_detect, callback_group=group)
        self.create_service(SetBool, '/atole/perception/set_stream', self._srv_stream, callback_group=group)
        self.create_service(Trigger, '/atole/perception/detector/warmup', self._srv_warmup, callback_group=group)
        self.stream_group = MutuallyExclusiveCallbackGroup()
        self.get_logger().info('detector_node listo')

    # ───────────────────────── modelos ─────────────────────────
    def _model(self, kind):
        if not self.config.wait_ready():
            raise RuntimeError('sin /atole/config')
        p = 'Perception/Eih' if kind == 'eih' else 'Perception/'
        wanted = MaskRCNN(self.config.get(f'{p}ModelPath', ''),
                          self.config.get(f'{p}ModelProfile', '') or self.config.get('Perception/ModelProfile', 'itabuna'),
                          self.config.get_float(f'{p}ScoreThresh', 0.5),
                          self.config.get('Perception/ExcludeClasses', '').split(','))
        current = self.models.get(kind)
        if current is None or current.key() != wanted.key():
            t0 = time.monotonic()
            wanted.load()
            self.models[kind] = current = wanted
            self.get_logger().info(f'Mask R-CNN {kind} cargado en {time.monotonic() - t0:.1f} s: {wanted.path} '
                                   f'(perfil {wanted.profile}, score ≥ {wanted.score_thresh})')
        return current

    # ───────────────────────── frames ─────────────────────────
    def _camera(self, cam):
        return cam or self.config.get('Cameras/SelectedEtH', 'cam1')

    def _latest_frame(self, cam, timeout=FRAME_TIMEOUT_S):
        if cam not in self.image_subs:
            self.frame_event[cam] = threading.Event()
            self.image_subs[cam] = self.create_subscription(
                Image, f'/atole/zed/{cam}/rgb/color/rect/image', lambda m, c=cam: self._on_image(c, m), IMAGE_QOS)
        self.frame_event[cam].clear()
        if not self.frame_event[cam].wait(timeout):
            raise TimeoutError(f'no llegan imágenes de /atole/zed/{cam}/rgb/color/rect/image en {timeout:.0f} s')
        return self.latest[cam]

    def _on_image(self, cam, msg):
        self.latest[cam] = msg
        self.frame_event[cam].set()

    # ───────────────────────── detección ─────────────────────────
    def detect(self, cam, image_msg, include_masks):
        bgr = to_bgr(image_msg)
        with self.infer_lock:                    # una inferencia a la vez en la GPU
            model = self._model('eih' if cam == 'cam2' else 'eth')
            dets, ms = model.run(bgr)
        h, w = bgr.shape[:2]
        out = DetectionArray(camera=cam, model_name=model.path.rsplit('/', 1)[-1], image_width=w, image_height=h,
                             inference_ms=float(ms))
        out.header = image_msg.header
        for i, d in enumerate(dets):
            det = Detection(id=i, class_name=d['class_name'], score=d['score'], bbox_xywh=d['bbox'])
            if include_masks:
                ys, xs = np.nonzero(d['mask'])
                x, y, bw, bh = d['bbox']
                x0, y0 = min(x, int(xs.min()) if len(xs) else x), min(y, int(ys.min()) if len(ys) else y)
                x1 = min(w, max(x + bw, int(xs.max()) + 1 if len(xs) else x + bw))
                y1 = min(h, max(y + bh, int(ys.max()) + 1 if len(ys) else y + bh))
                x0, y0 = max(0, x0), max(0, y0)
                det.mask_xy = [x0, y0]
                det.mask = mono8(d['mask'][y0:y1, x0:x1], image_msg.header.frame_id)
            out.detections.append(det)
        return out

    def _srv_detect(self, req, res):
        cam = self._camera(req.camera)
        try:
            image = req.image if req.image.data else self._latest_frame(cam)
            res.detections = self.detect(cam, image, req.include_masks)
        except Exception as e:
            res.ok, res.message = False, f'{type(e).__name__}: {e}'
            self.get_logger().error(f'detect_once {cam}: {res.message}')
            return res
        d = res.detections
        res.ok, res.message = True, f'{len(d.detections)} detecciones en {cam} ({d.inference_ms:.0f} ms)'
        return res

    def _srv_stream(self, req, res):
        if self.stream_timer is not None:
            self.destroy_timer(self.stream_timer)
            self.stream_timer = None
        if req.data:
            rate = max(0.2, self.config.get_float('Perception/StreamRateHz', 2.0))
            self.stream_timer = self.create_timer(1.0 / rate, self._stream_tick, callback_group=self.stream_group)
        res.success, res.message = True, 'detección continua ' + ('activada' if req.data else 'detenida')
        return res

    def _stream_tick(self):
        cam = self._camera('')
        try:
            self.det_pub.publish(self.detect(cam, self._latest_frame(cam, timeout=2.0), include_masks=True))
        except Exception as e:
            self.get_logger().warn(f'stream {cam}: {e}', throttle_duration_sec=10.0)

    def _srv_warmup(self, _req, res):
        parts = []
        try:
            for kind in ('eth', 'eih'):
                t0 = time.monotonic()
                with self.infer_lock:
                    self._model(kind).run(np.zeros((1536, 1920, 3), np.uint8))   # carga + autotune de cuDNN
                parts.append(f'{kind} {time.monotonic() - t0:.1f} s')
        except Exception as e:
            res.success, res.message = False, f'warm-up de Mask R-CNN falló: {type(e).__name__}: {e}'
            self.get_logger().error(res.message)
            return res
        res.success, res.message = True, 'Mask R-CNN ' + ' · '.join(parts)
        self.get_logger().info(f'warm-up: {res.message}')
        return res


def main(args=None):
    run_node(DetectorNode, args)


if __name__ == '__main__':
    main()
