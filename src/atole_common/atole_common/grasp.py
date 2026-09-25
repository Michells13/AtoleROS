"""Poses de pre-pick y pick de un pod (misma regla que PozoleV3: gui._approachFlip + harvest_sequence).

El Pod trae, en base_link, el punto de agarre, el eje de la base a la punta y una orientación cuyo Z
es ese eje. El gripper entra desde abajo a lo largo del eje:
  - Si el eje apunta hacia abajo (z < 0) se invierte, y la orientación gira 180° sobre su X local.
    En PozoleV3 el flag de usuario invert_normal se combinaba con esta regla por XOR y el resultado
    final era siempre el mismo: normal con z ≥ 0. Aquí se aplica la regla directamente.
  - Yaw del gripper (Harvest/GripperYaw) sobre el Z local del TCP.
  - pre-pick = g − n·ApproachDist ; pick = g + n·PickOffset, con g = grasp_bottom o grasp_midpoint
    según Harvest/PickPoint.
"""
import math

import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from scipy.spatial.transform import Rotation


def _xyz(v):
    return np.array([v.x, v.y, v.z], dtype=np.float64)


def targets(pod, harvest):
    """{'pre': (pos, quat_xyzw), 'pick': (pos, quat), 'normal': n, 'flipped': bool}.
    harvest = sección Harvest de Config.xml (dict de strings)."""
    g = _xyz(pod.grasp_midpoint if harvest.get('PickPoint', 'bottom') == 'midpoint' else pod.grasp_bottom)
    n = _xyz(pod.axis)
    n /= np.linalg.norm(n) + 1e-9
    o = pod.orientation
    rot = Rotation.from_quat([o.x, o.y, o.z, o.w])
    flipped = bool(n[2] < 0)
    if flipped:
        n = -n
        rot = rot * Rotation.from_rotvec([math.pi, 0.0, 0.0])
    rot = rot * Rotation.from_rotvec([0.0, 0.0, math.radians(float(harvest.get('GripperYaw', 0.0)))])
    q = rot.as_quat()
    approach = float(harvest.get('ApproachDist', 0.17))
    offset = float(harvest.get('PickOffset', 0.0))
    return {'pre': (g - n * approach, q), 'pick': (g + n * offset, q), 'normal': n, 'flipped': flipped}


def pose_stamped(pos, quat, frame='base_link'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
    msg.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
    return msg
