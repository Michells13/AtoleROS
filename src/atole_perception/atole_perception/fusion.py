"""Fusión de vistas EiH (PozoleV3 _voxel_merge, server.py:2360).

Las nubes se apilan en orden (la primera vista gana en cada vóxel), se indexan con floor(P / voxel)
—rejilla anclada en el origen del marco común— y se queda la primera aparición de cada vóxel,
manteniendo el orden original. Reproduce bit a bit Itabuna_EiH_Fused (20/20 carpetas).
"""
import numpy as np


def voxel_merge(clouds, voxel_m=0.003):
    """clouds: lista de (N_i, 3) en el mismo marco. Devuelve (M, 3) float64."""
    p = np.vstack([np.asarray(c, np.float64).reshape(-1, 3) for c in clouds])
    if not len(p):
        return p
    keys = np.floor(p / voxel_m).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return p[np.sort(idx)]


def to_frame(points, t_base_from_frame):
    """Puntos en base_link → marco cuya pose (base ← marco) es t_base_from_frame."""
    inv = np.linalg.inv(t_base_from_frame)
    return np.asarray(points, np.float64) @ inv[:3, :3].T + inv[:3, 3]
