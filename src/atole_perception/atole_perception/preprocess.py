"""Portado sin cambios de PozoleV3 (completion_methods/preprocess.py): mismo algoritmo, mismos parámetros.


Pre-completion preprocessing — replica EXACTA del pipeline que el proyecto POSE
aplica a la nube parcial ANTES de pasarla al modelo de completion (AdaPoinTr).

En POSE este preprocesamiento NO vive en el servidor de completion: el GUI
aplica una cadena de filtros (Voxel + SOR) y un denoiser (Bilateral) sobre la
nube parcial, y la nube resultante es la que se envía a `/completion`. El modelo
fue entrenado sobre parciales post-filtro+denoise, así que para que los
resultados de QuesadillaV1 coincidan con POSE hay que reproducir ese mismo
pipeline con los MISMOS algoritmos y parámetros.

Config replicada (validada contra la corrida de POSE, 1368 → 687 pts, -50%):

    Voxel(size_m=0.003)
      → SOR(k=20, std_ratio=2.0)
      → Bilateral(k=20, sigma_d_mm=5.0, sigma_n_mm=2.0, iters=1)

Las funciones `voxel`, `sor` y `bilateral` son copias byte-equivalentes de
`POSE/filters.py` y `POSE/denoise_methods/bilateral.py` (numpy + scipy puro, sin
deps compiladas) para garantizar coincidencia bit a bit. No usar los filtros
open3d de QuesadillaV1 aquí: dan resultados distintos.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


# ─── Filtros (clonados de POSE/filters.py) ───────────────────────────────────

def voxel(xyz: np.ndarray, size_m: float = 0.003) -> np.ndarray:
    """Downsample: un punto representativo (primera ocurrencia en orden de
    escaneo) por voxel de lado size_m. Idéntico a POSE."""
    if xyz.shape[0] == 0 or size_m <= 0:
        return xyz
    keys = np.floor(xyz / size_m).astype(np.int64)
    off = (keys.min(axis=0) - 1).astype(np.int64)
    keys = keys - off
    code = (keys[:, 0].astype(np.int64) << 42
            | keys[:, 1].astype(np.int64) << 21
            | keys[:, 2].astype(np.int64))
    _, first_idx = np.unique(code, return_index=True)
    mask = np.zeros(xyz.shape[0], dtype=bool)
    mask[first_idx] = True
    return xyz[mask]


def sor(xyz: np.ndarray, k: int = 20, std_ratio: float = 2.0) -> np.ndarray:
    """Statistical Outlier Removal. Idéntico a POSE."""
    if xyz.shape[0] < k + 1:
        return xyz
    tree = cKDTree(xyz)
    dists, _ = tree.query(xyz, k=k + 1)
    mean_dists = dists[:, 1:].mean(axis=1)
    mu = float(mean_dists.mean())
    sig = float(mean_dists.std()) + 1e-9
    mask = mean_dists < (mu + std_ratio * sig)
    return xyz[mask]


# ─── Denoise bilateral (clonado de POSE/denoise_methods/bilateral.py) ────────

def bilateral(xyz: np.ndarray, k: int = 20, sigma_d_mm: float = 5.0,
              sigma_n_mm: float = 2.0, iters: int = 1) -> np.ndarray:
    """Filtro bilateral (Fleishman 2003). Mismo número de puntos a la salida.
    Idéntico a POSE/denoise_methods/bilateral.py."""
    pts = np.asarray(xyz, dtype=np.float64)
    if pts.shape[0] < 10:
        return pts.astype(np.float32)

    sigma_d = float(sigma_d_mm) / 1000.0
    sigma_n = float(sigma_n_mm) / 1000.0

    cur = pts.copy()
    for _ in range(int(iters)):
        tree = cKDTree(cur)
        _, idx = tree.query(cur, k=min(int(k) + 1, cur.shape[0]))
        idx = idx[:, 1:]
        normals = np.empty_like(cur)
        for i in range(cur.shape[0]):
            nbr = cur[idx[i]]
            c = nbr.mean(axis=0)
            _, _, Vt = np.linalg.svd(nbr - c, full_matrices=False)
            normals[i] = Vt[-1]
        new_cur = np.empty_like(cur)
        sd2 = 2.0 * sigma_d * sigma_d
        sn2 = 2.0 * sigma_n * sigma_n
        for i in range(cur.shape[0]):
            nbr = cur[idx[i]]
            diff = nbr - cur[i]
            r2 = (diff ** 2).sum(axis=1)
            d = diff @ normals[i]
            w = np.exp(-r2 / sd2) * np.exp(-(d ** 2) / sn2)
            wsum = float(w.sum() + 1e-12)
            delta = float((w * d).sum() / wsum)
            new_cur[i] = cur[i] + delta * normals[i]
        cur = new_cur
    return cur.astype(np.float32)


# ─── Pipeline completo ───────────────────────────────────────────────────────

# Parámetros por defecto = config validada de POSE.
DEFAULTS = {
    "voxel_size_m":     0.003,
    "sor_k":            20,
    "sor_std_ratio":    2.0,
    "bilateral_k":      20,
    "bilateral_sigma_d_mm": 5.0,
    "bilateral_sigma_n_mm": 2.0,
    "bilateral_iters":  1,
}


def preprocess_for_completion(xyz: np.ndarray, params: dict | None = None
                              ) -> tuple[np.ndarray, dict]:
    """
    Aplica Voxel → SOR → Bilateral (mismos algoritmos/params que POSE) sobre la
    nube parcial, en frame cámara / metros. Retorna (xyz_float32, stats).

    El orden replica POSE: la cadena de filtros (voxel, luego sor) corre primero
    y el denoise bilateral va al final sobre la nube ya filtrada.
    """
    p = {**DEFAULTS, **(params or {})}
    pts = np.asarray(xyz, dtype=np.float64)
    n0 = int(pts.shape[0])

    pts = voxel(pts, size_m=float(p["voxel_size_m"]))
    n1 = int(pts.shape[0])

    pts = sor(pts, k=int(p["sor_k"]), std_ratio=float(p["sor_std_ratio"]))
    n2 = int(pts.shape[0])

    pts = bilateral(pts, k=int(p["bilateral_k"]),
                    sigma_d_mm=float(p["bilateral_sigma_d_mm"]),
                    sigma_n_mm=float(p["bilateral_sigma_n_mm"]),
                    iters=int(p["bilateral_iters"]))
    n3 = int(pts.shape[0])

    stats = {"n_in": n0, "n_voxel": n1, "n_sor": n2, "n_out": n3}
    return pts.astype(np.float32), stats
