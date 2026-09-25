"""Matemática del refinamiento EiH, portada sin cambios de PozoleV3 (server.py).

- correct_pod: _eih_build_update (1574). La posición se reemplaza por la estimada con cam2 (sin
  mezcla); el eje nuevo se guarda tal cual y la orientación es la rotación MÍNIMA que lleva el eje
  EtH al nuevo, limitada a cap_deg y aplicada sobre la orientación EtH (conserva el giro del gripper).
- ppp_pose: _eih_compute_ppp_tcp (2527). Pre-pre-pick a la misma altura que el pre-pick, desplazado
  offset_m en XY hacia Home2, e inclinado lo mínimo para que la cámara vuelva a apuntar al pod
  (eje óptico empírico derivado del pre-pick), con la inclinación limitada a max_tilt_deg.
"""
import copy
import math

import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from scipy.spatial.transform import Rotation


def _xyz(v):
    return np.array([v.x, v.y, v.z], dtype=np.float64)


def correct_pod(pod_eth, pod_eih, cap_deg=15.0):
    """(pod corregido, info). El pod corregido conserva id/score del EtH."""
    n0 = _xyz(pod_eth.axis)
    n0 /= np.linalg.norm(n0) + 1e-9
    n_new = _xyz(pod_eih.axis)
    n_new /= np.linalg.norm(n_new) + 1e-9
    r_delta = Rotation.align_vectors([n_new], [n0])[0]
    rotvec = r_delta.as_rotvec()
    ang = math.degrees(float(np.linalg.norm(rotvec)))
    capped = cap_deg > 0 and ang > cap_deg
    if capped:
        r_delta = Rotation.from_rotvec(rotvec * (cap_deg / ang))
    o = pod_eth.orientation
    q_new = (r_delta * Rotation.from_quat([o.x, o.y, o.z, o.w])).as_quat()
    pod = copy.deepcopy(pod_eth)
    pod.grasp_bottom = copy.deepcopy(pod_eih.grasp_bottom)
    pod.grasp_midpoint = copy.deepcopy(pod_eih.grasp_midpoint)
    pod.tip = copy.deepcopy(pod_eih.tip)
    pod.centroid = copy.deepcopy(pod_eih.centroid)
    pod.axis = Vector3(x=float(n_new[0]), y=float(n_new[1]), z=float(n_new[2]))
    pod.orientation = Quaternion(x=float(q_new[0]), y=float(q_new[1]), z=float(q_new[2]), w=float(q_new[3]))
    info = {'correction_m': float(np.linalg.norm(_xyz(pod_eih.grasp_bottom) - _xyz(pod_eth.grasp_bottom))),
            'angle_deg': min(ang, cap_deg) if cap_deg > 0 else ang, 'angle_raw_deg': ang, 'capped': capped}
    return pod, info


def _rot_between(a, b):
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    s, c = float(np.linalg.norm(v)), float(np.dot(a, b))
    if s < 1e-9:
        return np.eye(3)
    return Rotation.from_rotvec(v / s * math.atan2(s, c)).as_matrix()


def ppp_pose(prepick_pos, prepick_quat, target, home2_xy, t_tcp_cam, offset_m=0.10, toward_home2=True,
             aim=True, max_tilt_deg=35.0):
    """(pos, quat_xyzw, info) del TCP en el pre-pre-pick. t_tcp_cam: 4×4 TCP ← cámara."""
    prepick_pos = np.asarray(prepick_pos, np.float64)
    r_pp = Rotation.from_quat(np.asarray(prepick_quat, np.float64)).as_matrix()
    d_xy = prepick_pos[:2] - np.asarray(home2_xy, np.float64)[:2]
    nrm = float(np.linalg.norm(d_xy))
    if nrm < 1e-4:
        raise ValueError('PPP: Home2 y el pre-pick coinciden en XY (no hay dirección)')
    d_xy /= nrm
    sgn = -1.0 if toward_home2 else 1.0
    pos = prepick_pos.copy()
    pos[:2] += sgn * offset_m * d_xy
    info = {'tilt_deg': 0.0, 'clamped': False}
    if not aim or target is None:
        return pos, np.asarray(prepick_quat, np.float64), info
    c = np.asarray(target, np.float64)
    r_x, t_x = t_tcp_cam[:3, :3], t_tcp_cam[:3, 3]
    r_cam_pp = r_pp @ r_x
    look = c - (prepick_pos + r_pp @ t_x)
    if np.linalg.norm(look) < 1e-6:
        return pos, np.asarray(prepick_quat, np.float64), info
    a_opt = r_cam_pp.T @ (look / np.linalg.norm(look))
    r_cam = r_cam_pp.copy()
    for _ in range(6):                      # el brazo de palanca cámara↔TCP obliga a iterar
        z_des = c - (pos + (r_cam @ r_x.T) @ t_x)
        z_des /= np.linalg.norm(z_des) + 1e-12
        r_cam = _rot_between(r_cam @ a_opt, z_des) @ r_cam
    r_tcp = r_cam @ r_x.T
    rotvec = Rotation.from_matrix(r_tcp @ r_pp.T).as_rotvec()
    ang = float(np.linalg.norm(rotvec))
    max_tilt = math.radians(max_tilt_deg)
    if ang > max_tilt and ang > 1e-9:
        r_tcp = Rotation.from_rotvec(rotvec / ang * max_tilt).as_matrix() @ r_pp
        info['clamped'], ang = True, max_tilt
    info['tilt_deg'] = math.degrees(ang)
    return pos, Rotation.from_matrix(r_tcp).as_quat(), info

