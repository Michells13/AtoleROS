"""dataset_player — modo SIM: reproduce una captura EtH guardada como si fueran las cámaras.

Usa el mismo dataset que PozoleV3 (Sim/DatasetDir = Itabuna_plus) y, por ahora, solo las
cámaras EtH cam0 y cam1: no hay dataset de cam2 con su pose en el espacio. Publica, por cada
una, los mismos topics, marcos y codificaciones que zed-ros2-wrapper:
    /atole/zed/<cam>/rgb/color/rect/image        bgra8
    /atole/zed/<cam>/depth/depth_registered       32FC1 (metros)
    /atole/zed/<cam>/confidence/confidence_map    32FC1 (0 = máxima confianza)
    /atole/zed/<cam>/rgb/color/rect/camera_info   intrínsecos rectificados, D = 0
    /atole/zed/<cam>/status/heartbeat             (camera_manager la ve como "transmitiendo")
frame_id = <cam>_left_camera_frame_optical; todos los mensajes de un ciclo llevan el mismo stamp.

Formato de carpeta: <cam>_sn<serial>_rgb.png, _depth_metric.npy (o Z de _pointcloud.npy),
_confidence.npy si existe, y calibration.json con los intrínsecos de esa captura (la ZED se
autocalibra: cambian ligeramente entre carpetas). PozoleV3 lee directamente _pointcloud.npy;
profundidad + estos intrínsecos reproducen esa nube con < 0.003 mm de diferencia (float32).
"""
import json
from pathlib import Path

import cv2
import numpy as np
from atole_interfaces.srv import ListDatasets, LoadDataset
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from zed_msgs.msg import Heartbeat

from atole_common.config_view import ConfigView
from atole_common.stubs import run_node

SIM_CAMERAS = ('cam0', 'cam1')      # cam2 (EiH) queda fuera hasta tener datasets con su pose


def rgb_file(folder, cam):
    return next(iter(sorted(folder.glob(f'{cam}_*_rgb.png'))), None)     # excluye *_rgb_right.png


def image_msg(array, encoding, frame):
    msg = Image(height=array.shape[0], width=array.shape[1], encoding=encoding, is_bigendian=0)
    msg.step = array.strides[0]
    msg.data = np.ascontiguousarray(array).tobytes()
    msg.header.frame_id = frame
    return msg


class DatasetPlayer(Node):

    def __init__(self):
        super().__init__('dataset_player')
        self.frames = {}        # cam -> {'rgb': Image, 'depth': Image, 'conf': Image, 'info': CameraInfo, 'serial': int}
        self.pubs = {}
        self.timer = None
        self.folder = ''
        self.beat = 0
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.config = ConfigView(self)
        self.create_service(ListDatasets, '/atole/sim/list', self._srv_list)
        self.create_service(LoadDataset, '/atole/sim/load', self._srv_load)
        self.create_service(Trigger, '/atole/sim/stop', self._srv_stop)
        self.get_logger().info('dataset_player listo (modo SIM): /atole/sim/list y /atole/sim/load')

    def _root(self):
        return Path(self.config.get('Sim/DatasetDir', '')).expanduser()

    def _srv_list(self, _req, res):
        root = self._root()
        res.root = str(root)
        if root.is_dir():
            res.folders = sorted((d.name for d in root.iterdir()
                                  if d.is_dir() and any(rgb_file(d, cam) for cam in SIM_CAMERAS)), reverse=True)
        return res

    def _load(self, folder):
        calib = {}
        cj = folder / 'calibration.json'
        if cj.exists():
            for cam in json.loads(cj.read_text()).get('cameras', []):
                calib[cam.get('label')] = cam
        frames = {}
        for cam in SIM_CAMERAS:
            rgb_path = rgb_file(folder, cam)
            if rgb_path is None:
                continue
            prefix = rgb_path.name[:-len('rgb.png')]
            intr = calib.get(cam, {}).get('intrinsics', {})
            if not intr:
                self.get_logger().error(f'{folder.name}/{cam}: calibration.json no tiene intrínsecos; se omite')
                continue
            bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            depth_file = folder / f'{prefix}depth_metric.npy'
            if depth_file.exists():
                depth = np.load(depth_file).astype(np.float32)
            else:
                depth = np.load(folder / f'{prefix}pointcloud.npy')[..., 2].astype(np.float32)
            conf_file = folder / f'{prefix}confidence.npy'
            if conf_file.exists():
                conf = np.load(conf_file).astype(np.float32)
            else:
                conf = np.zeros(depth.shape, np.float32)
                self.get_logger().warn(f'{folder.name}/{cam}: sin confidence.npy; se publica confianza 0 (máxima)')
            frame = f'{cam}_left_camera_frame_optical'
            h, w = depth.shape
            info = CameraInfo(height=h, width=w, distortion_model='plumb_bob', d=[0.0] * 5)
            fx, fy, cx, cy = (float(intr[k]) for k in ('fx', 'fy', 'cx', 'cy'))
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.header.frame_id = frame
            frames[cam] = {'rgb': image_msg(cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA), 'bgra8', frame),
                           'depth': image_msg(depth, '32FC1', frame), 'conf': image_msg(conf, '32FC1', frame),
                           'info': info, 'serial': int(calib.get(cam, {}).get('serial', 0) or 0)}
        return frames

    def _cam_publishers(self, cam):
        if cam not in self.pubs:
            base = f'/atole/zed/{cam}'
            self.pubs[cam] = {
                'rgb': self.create_publisher(Image, f'{base}/rgb/color/rect/image', self.qos),
                'depth': self.create_publisher(Image, f'{base}/depth/depth_registered', self.qos),
                'conf': self.create_publisher(Image, f'{base}/confidence/confidence_map', self.qos),
                'info': self.create_publisher(CameraInfo, f'{base}/rgb/color/rect/camera_info', self.qos),
                'beat': self.create_publisher(Heartbeat, f'{base}/status/heartbeat', 10),
            }
        return self.pubs[cam]

    def _srv_load(self, req, res):
        folder = self._root() / req.folder
        if not folder.is_dir():
            res.ok, res.message = False, f'no existe {folder}'
            return res
        frames = self._load(folder)
        if not frames:
            res.ok, res.message = False, f'{req.folder}: no hay imágenes de cámara utilizables'
            return res
        if self.timer is not None:
            self.destroy_timer(self.timer)
        self.frames, self.folder = frames, req.folder
        rate = req.rate_hz or self.config.get_float('Sim/RateHz', 2.0)
        self.timer = self.create_timer(1.0 / max(0.1, rate), self._tick)
        res.ok, res.message = True, f'{req.folder}: publicando {sorted(frames)} a {rate:.1f} Hz'
        self.get_logger().info(res.message)
        return res

    def _srv_stop(self, _req, res):
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        res.success, res.message = True, f'reproducción detenida ({self.folder or "nada cargado"})'
        return res

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        self.beat += 1
        for cam, f in self.frames.items():
            pubs = self._cam_publishers(cam)
            for key in ('rgb', 'depth', 'conf', 'info'):
                f[key].header.stamp = stamp
                pubs[key].publish(f[key])
            pubs['beat'].publish(Heartbeat(beat_count=self.beat, node_name='dataset_player',
                                           camera_sn=f['serial'], simul_mode=True))


def main(args=None):
    run_node(DatasetPlayer, args)


if __name__ == '__main__':
    main()
