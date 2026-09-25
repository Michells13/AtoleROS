"""Conversiones entre mensajes ROS y numpy para la percepción."""
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Vector3
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2, PointField

CHANNELS = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3, 'mono8': 1, '32FC1': 1}


def image_to_array(msg):
    """sensor_msgs/Image → ndarray (respeta el step). 32FC1 → float32 H×W; el resto uint8."""
    if msg.encoding not in CHANNELS:
        raise ValueError(f'codificación no soportada: {msg.encoding}')
    ch = CHANNELS[msg.encoding]
    dtype = np.float32 if msg.encoding == '32FC1' else np.uint8
    item = np.dtype(dtype).itemsize
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch * item]
    arr = rows.copy().view(dtype).reshape(msg.height, msg.width, ch)
    return arr[..., 0] if ch == 1 else arr


def to_bgr(msg):
    arr = image_to_array(msg)
    if msg.encoding in ('bgra8', 'bgr8'):
        return np.ascontiguousarray(arr[..., :3])
    if msg.encoding in ('rgba8', 'rgb8'):
        return np.ascontiguousarray(arr[..., 2::-1])
    raise ValueError(f'se esperaba una imagen de color, llegó {msg.encoding}')


def mono8(array, frame_id='', stamp=None):
    msg = Image(height=array.shape[0], width=array.shape[1], encoding='mono8', is_bigendian=0, step=array.shape[1])
    msg.data = np.ascontiguousarray(array, np.uint8).tobytes()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    return msg


def transform_to_matrix(t):
    m = np.eye(4)
    q = t.rotation
    m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def vector(v):
    return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def quaternion(q):
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def cloud_msg(xyz, frame_id, stamp, ids=None):
    """PointCloud2 xyz (float32) con un campo opcional `id` (float32) para distinguir pods."""
    xyz = np.asarray(xyz, np.float32).reshape(-1, 3)
    names = ['x', 'y', 'z'] + (['id'] if ids is not None else [])
    data = xyz if ids is None else np.hstack([xyz, np.asarray(ids, np.float32).reshape(-1, 1)])
    msg = PointCloud2(height=1, width=len(data), is_bigendian=False, is_dense=True,
                      point_step=4 * len(names), row_step=4 * len(names) * len(data))
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(names)]
    msg.data = np.ascontiguousarray(data, np.float32).tobytes()
    msg.header.frame_id, msg.header.stamp = frame_id, stamp
    return msg
