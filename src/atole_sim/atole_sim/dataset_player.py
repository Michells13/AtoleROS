"""dataset_player — modo SIM: reproduce capturas guardadas como si fueran las cámaras.

EtH: el mismo dataset que PozoleV3 (Sim/DatasetDir = Itabuna_plus), cámaras cam0 y cam1.
EiH: vistas de cam2 con la pose del TCP grabada (Sim/EihRoot; /atole/sim/list_eih y load_eih),
con su TF base_link → cam2_calibrated_optical. Publica, por cada cámara, los mismos topics,
marcos y codificaciones que zed-ros2-wrapper:
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
import re
from pathlib import Path

import cv2
import numpy as np
from atole_interfaces.srv import ListDatasets, LoadDataset
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger

from zed_msgs.msg import Heartbeat

from atole_common.config_view import ConfigView
from atole_common.static_tf import StaticTf
from atole_common.stubs import run_node

SIM_CAMERAS = ('cam0', 'cam1')      # EtH; cam2 (EiH) va por /atole/sim/load_eih
EIH_VIEW = re.compile(r'^cam2_(eih\d+)_rgb\.png$')


def intrinsics_from_cloud(cloud):
    """fx, fy, cx, cy de una nube organizada (H×W×3|4, marco óptico): u = fx·X/Z + cx, v = fy·Y/Z + cy.
    Exacto para las nubes de la ZED (residuo < 0.001 px)."""
    h, w = cloud.shape[:2]
    v, u = np.mgrid[0:h, 0:w]
    x, y, z = cloud[..., 0], cloud[..., 1], cloud[..., 2]
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > 0)
    fx, cx = np.polyfit(x[ok] / z[ok], u[ok], 1)
    fy, cy = np.polyfit(y[ok] / z[ok], v[ok], 1)
    return float(fx), float(fy), float(cx), float(cy)


def tcp_to_matrix(tcp):
    """Pose del AUBO [x, y, z, roll, pitch, yaw] (RPY intrínseco ZYX) → 4×4."""
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler('ZYX', [tcp[5], tcp[4], tcp[3]]).as_matrix()
    m[:3, 3] = tcp[:3]
    return m


def to_transform(m, parent, child, stamp):
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in m[:3, 3])
    x, y, z, w = Rotation.from_matrix(m[:3, :3]).as_quat()
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = x, y, z, w
    return t


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
        self.create_service(ListDatasets, '/atole/sim/list_eih', self._srv_list_eih)
        self.create_service(LoadDataset, '/atole/sim/load_eih', self._srv_load_eih)
        self.tf_static = StaticTf(self)
        self.get_logger().info('dataset_player listo (modo SIM): /atole/sim/list|load (EtH) y list_eih|load_eih (cam2)')

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
            k = tuple(float(intr[key]) for key in ('fx', 'fy', 'cx', 'cy'))
            frames[cam] = self._frames(cam, bgr, depth, conf, k, int(calib.get(cam, {}).get('serial', 0) or 0))
        return frames

    @staticmethod
    def _frames(cam, bgr, depth, conf, k, serial):
        """Mensajes de un frame con los mismos topics, marcos y codificaciones que la ZED."""
        fx, fy, cx, cy = k
        frame = f'{cam}_left_camera_frame_optical'
        h, w = depth.shape
        info = CameraInfo(height=h, width=w, distortion_model='plumb_bob', d=[0.0] * 5)
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.header.frame_id = frame
        return {'rgb': image_msg(cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA), 'bgra8', frame),
                'depth': image_msg(depth, '32FC1', frame), 'conf': image_msg(conf, '32FC1', frame),
                'info': info, 'serial': serial}

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
        self.frames = {**{c: f for c, f in self.frames.items() if c not in SIM_CAMERAS}, **frames}   # conserva cam2
        self.folder = req.folder
        rate = self._start_timer(req.rate_hz)
        res.ok, res.message = True, f'{req.folder}: publicando {sorted(frames)} a {rate:.1f} Hz'
        self.get_logger().info(res.message)
        return res

    def _start_timer(self, rate_hz):
        if self.timer is not None:
            self.destroy_timer(self.timer)
        rate = rate_hz or self.config.get_float('Sim/RateHz', 2.0)
        self.timer = self.create_timer(1.0 / max(0.1, rate), self._tick)
        return rate

    # ───────────────────────── cam2 (EiH) ─────────────────────────
    # Vistas con la pose del TCP grabada, relativas a Sim/EihRoot:
    #   Itabuna_EiH/<ts>                 (rgb.png, depth_metric.npy, confidence.npy, metadata.json, calibration.json)
    #   Itabuna_plus/<ts>/eihNN          (cam2_eihNN_rgb.png, …, cam2_eihNN_calibration.json)
    # Se publica cam2 como la ZED y el TF base_link → cam2_calibrated_optical con
    # T = TCP grabado · calibración de cam2 en Config.xml (extrinsics_publisher no publica cam2 en SIM).
    def _eih_root(self):
        return Path(self.config.get('Sim/EihRoot', '')).expanduser()

    def _eih_views(self):
        root = self._eih_root()
        views = []
        for cal in sorted(root.glob('*/*/calibration.json')):
            if json.loads(cal.read_text()).get('tcp_pose_at_capture') is not None:
                views.append(str(cal.parent.relative_to(root)))
        for cal in sorted(root.glob('*/*/cam2_eih*_calibration.json')):
            if json.loads(cal.read_text()).get('tcp_pose_at_capture') is not None:
                views.append(f'{cal.parent.relative_to(root)}/{cal.name.split("_")[1]}')
        return views

    def _srv_list_eih(self, _req, res):
        res.root = str(self._eih_root())
        res.folders = self._eih_views()
        return res

    def _load_eih_view(self, view):
        root = self._eih_root()
        path = root / view
        if path.is_dir():                                         # Itabuna_EiH/<ts>
            files = {k: path / n for k, n in (('rgb', 'rgb.png'), ('depth', 'depth_metric.npy'), ('conf', 'confidence.npy'),
                                             ('cloud', 'pointcloud.npy'), ('calib', 'calibration.json'))}
            meta = path / 'metadata.json'
            intr = json.loads(meta.read_text()).get('intrinsics', {}).get('left') if meta.exists() else None
        else:                                                     # Itabuna_plus/<ts>/eihNN
            folder, tag = path.parent, path.name
            files = {k: folder / f'cam2_{tag}_{n}' for k, n in (('rgb', 'rgb.png'), ('depth', 'depth_metric.npy'),
                     ('conf', 'confidence.npy'), ('cloud', 'pointcloud.npy'), ('calib', 'calibration.json'))}
            intr = None
        for key in ('rgb', 'calib'):
            if not files[key].exists():
                raise FileNotFoundError(f'no existe {files[key]}')
        calib = json.loads(files['calib'].read_text())
        tcp = calib.get('tcp_pose_at_capture')
        if tcp is None:
            raise ValueError(f'{view}: la captura no tiene la pose del TCP')
        if files['depth'].exists():
            depth = np.load(files['depth']).astype(np.float32)
        else:
            depth = np.load(files['cloud'])[..., 2].astype(np.float32)
        conf = np.load(files['conf']).astype(np.float32) if files['conf'].exists() else np.zeros(depth.shape, np.float32)
        if intr:
            fx, fy, cx, cy = (float(intr[k]) for k in ('fx', 'fy', 'cx', 'cy'))
        else:
            fx, fy, cx, cy = intrinsics_from_cloud(np.load(files['cloud']))
        matrix = self.config.get('Calibrations/cam2/Matrix', '').split()
        t_tcp_cam = (np.array([float(v) for v in matrix]).reshape(4, 4) if len(matrix) == 16
                     else np.array(calib['T_tcp_to_camera_cam2']).reshape(4, 4))
        bgr = cv2.imread(str(files['rgb']), cv2.IMREAD_COLOR)
        frame = self._frames('cam2', bgr, depth, conf, (fx, fy, cx, cy), int(calib.get('serial_number', 0) or 0))
        return frame, tcp_to_matrix(tcp) @ t_tcp_cam

    def _srv_load_eih(self, req, res):
        try:
            frame, t_base_cam = self._load_eih_view(req.folder)
        except Exception as e:
            res.ok, res.message = False, f'{type(e).__name__}: {e}'
            return res
        self.frames['cam2'] = frame
        stamp = self.get_clock().now().to_msg()
        self.tf_static.sendTransform([to_transform(t_base_cam, 'base_link', 'cam2_calibrated_optical', stamp),
                                      to_transform(t_base_cam, 'base_link', 'cam2_left_camera_frame_optical', stamp)])
        rate = self._start_timer(req.rate_hz)
        res.ok = True
        res.message = f'{req.folder}: cam2 en {np.round(t_base_cam[:3, 3], 3).tolist()} (base_link) a {rate:.1f} Hz'
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
