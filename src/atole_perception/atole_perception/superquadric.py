"""Portado sin cambios de PozoleV3 (superquadric_pose.py): mismo algoritmo, mismos parámetros.


superquadric_pose.py
====================
Ajuste de un **superquadric** (formulación Solina-Bajcsy, variante pod ε1=1) a
una nube de puntos y muestreo de su superficie, para estimar la pose vía PCA.

Portado del proyecto POSE (`pose_methods/superquadric.py`). En POSE el eje se
obtenía corriendo PCA sobre la superficie muestreada del superquadric
(`_axis_helper.longitudinal_markers`). Aquí se devuelve la superficie muestreada
y se deja que `pose_estimator.estimate_pose` corra el PCA — así la pose resultante
usa EXACTAMENTE la misma convención (eje, quaternion, width/offset, confidence)
que los otros métodos de QuesadillaV1, y fluye por el mismo pipeline robot/markers.

Flujo "superquadric → PCA":
    fit_and_sample(points) → superficie cerrada (M,3) → estimate_pose(superficie)

Coordenadas: frame ZED (X right, Y down, Z forward), metros — igual que la entrada.
"""

from __future__ import annotations

import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# Defaults (de POSE/pose_methods/superquadric.py PARAMS). Variante pod: ε1=1.
PARAMS = {
    "max_pts":  1500,    # subsample para el ajuste LM
    "max_iter": 300,     # iteraciones máximas LM
    "a_min_mm": 5.0,     # cota inferior semieje
    "a_max_mm": 300.0,   # cota superior semieje
    "e_min":    0.10,    # cota inferior exponente
    "e_max":    2.00,    # cota superior exponente
    "n_eta":    24,      # resolución muestreo (latitudes)
    "n_omega":  48,      # resolución muestreo (longitudes)
}


# ─── Math helpers (idénticos a POSE) ─────────────────────────────────────────

def _sgnpow(v: np.ndarray, p: float) -> np.ndarray:
    av = np.abs(v)
    return np.sign(v) * (av ** p)


def _F(p_local: np.ndarray, a: tuple, e: tuple) -> np.ndarray:
    a1, a2, a3 = a
    e1, e2     = e
    x = p_local[:, 0] / a1
    y = p_local[:, 1] / a2
    z = p_local[:, 2] / a3
    Axy = (np.abs(x) ** (2.0 / e2) + np.abs(y) ** (2.0 / e2)) ** (e2 / e1)
    Az  = np.abs(z) ** (2.0 / e1)
    return Axy + Az


def _world_to_local(points: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (points - t) @ R


def _initial_guess_from_pca(points: np.ndarray) -> tuple:
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, Vt = np.linalg.svd(centered, full_matrices=False)
    R0 = Vt.T
    if np.linalg.det(R0) < 0:
        R0[:, -1] *= -1
    proj = centered @ R0
    a0 = np.maximum(0.5 * (proj.max(axis=0) - proj.min(axis=0)), 0.005)
    return R0, centroid, a0


def _residuals(theta: np.ndarray, points: np.ndarray) -> np.ndarray:
    # Variante pod: ε1 fijo en 1.0. theta = [a1,a2,a3, e2, rx,ry,rz, tx,ty,tz]
    a1, a2, a3, e2, rx, ry, rz, tx, ty, tz = theta
    e1 = 1.0
    R = Rotation.from_euler("xyz", [rx, ry, rz]).as_matrix()
    t = np.array([tx, ty, tz])
    p_local = _world_to_local(points, R, t)
    F = _F(p_local, (a1, a2, a3), (e1, e2))
    vol_w = np.sqrt(a1 * a2 * a3)
    return vol_w * (np.power(F, e1) - 1.0)


def _fit_superquadric(points: np.ndarray, params: dict) -> dict:
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 50:
        return {"success": False, "error_msg": f"too few points: {pts.shape[0]}"}

    max_pts = int(params["max_pts"])
    if pts.shape[0] > max_pts:
        idx = np.linspace(0, pts.shape[0] - 1, max_pts, dtype=int)
        pts_fit = pts[idx]
    else:
        pts_fit = pts

    R0, t0, a0 = _initial_guess_from_pca(pts_fit)
    rx0, ry0, rz0 = Rotation.from_matrix(R0).as_euler("xyz")

    a_lo = float(params["a_min_mm"]) / 1000.0
    a_hi = float(params["a_max_mm"]) / 1000.0
    e_lo = float(params["e_min"])
    e_hi = float(params["e_max"])

    theta0 = np.array([a0[0], a0[1], a0[2], 1.0,
                       rx0, ry0, rz0, t0[0], t0[1], t0[2]])
    lb = [a_lo, a_lo, a_lo, e_lo, -np.pi*2, -np.pi*2, -np.pi*2, -2.0, -2.0, 0.05]
    ub = [a_hi, a_hi, a_hi, e_hi,  np.pi*2,  np.pi*2,  np.pi*2,  2.0,  2.0, 20.0]
    theta0 = np.clip(theta0, lb, ub)

    try:
        sol = least_squares(
            _residuals, theta0, args=(pts_fit,),
            bounds=(lb, ub), method="trf", x_scale="jac",
            max_nfev=int(params["max_iter"]), ftol=1e-6, xtol=1e-6,
        )
    except Exception as e:
        return {"success": False, "error_msg": f"LM failed: {type(e).__name__}: {e}"}

    a1, a2, a3, e2, rx, ry, rz, tx, ty, tz = sol.x
    e1 = 1.0
    R = Rotation.from_euler("xyz", [rx, ry, rz]).as_matrix()
    t = np.array([tx, ty, tz])

    p_local_full = _world_to_local(pts, R, t)
    F_full = _F(p_local_full, (a1, a2, a3), (e1, e2))
    res_full = np.sqrt(a1 * a2 * a3) * (np.power(F_full, e1) - 1.0)
    rms = float(np.sqrt(np.mean(res_full ** 2)))

    return {"success": True, "a": (float(a1), float(a2), float(a3)),
            "e": (float(e1), float(e2)), "R": R, "t": t,
            "rms": rms, "n_iters": int(sol.nfev)}


def _sample_surface(a: tuple, e: tuple, R: np.ndarray, t: np.ndarray,
                    n_eta: int, n_omega: int) -> np.ndarray:
    a1, a2, a3 = a
    e1, e2     = e
    eta   = np.linspace(-np.pi/2, np.pi/2, n_eta)
    omega = np.linspace(-np.pi, np.pi, n_omega)
    EE, OO = np.meshgrid(eta, omega, indexing="ij")
    cos_e = _sgnpow(np.cos(EE), e1)
    sin_e = _sgnpow(np.sin(EE), e1)
    cos_o = _sgnpow(np.cos(OO), e2)
    sin_o = _sgnpow(np.sin(OO), e2)
    x = a1 * cos_e * cos_o
    y = a2 * cos_e * sin_o
    z = a3 * sin_e
    pts_local = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    return pts_local @ R.T + t


def fit_and_sample(points: np.ndarray, params: dict | None = None) -> dict:
    """
    Ajusta un superquadric (pod, ε1=1) a `points` y muestrea su superficie.

    Retorna dict:
        success, error_msg,
        surface_xyz : (n_eta*n_omega, 3) float32 — superficie cerrada muestreada,
                      mismo frame/units que la entrada. Alimentar a estimate_pose.
        metrics     : {a_mm:[a1,a2,a3], epsilon:[e1,e2], rms_mm, length_mm, n_iters}
        elapsed_ms
    """
    t0 = time.perf_counter()
    p = {**PARAMS, **(params or {})}

    fit = _fit_superquadric(np.asarray(points, dtype=np.float64), p)
    if not fit["success"]:
        return {"success": False, "error_msg": fit.get("error_msg", "fit failed"),
                "surface_xyz": None, "metrics": {},
                "elapsed_ms": (time.perf_counter() - t0) * 1000}

    a, e = fit["a"], fit["e"]
    surf = _sample_surface(a, e, fit["R"], fit["t"],
                           int(p["n_eta"]), int(p["n_omega"])).astype(np.float32)

    # length_mm = extensión de la superficie a lo largo de su eje principal (PCA),
    # solo informativo (la pose real la calcula estimate_pose sobre la superficie).
    c = surf.mean(axis=0)
    _, _, Vt = np.linalg.svd(surf - c, full_matrices=False)
    proj = (surf - c) @ Vt[0]
    length_mm = round(float(proj.max() - proj.min()) * 1000, 1)

    elapsed = (time.perf_counter() - t0) * 1000
    return {
        "success": True, "error_msg": "",
        "surface_xyz": surf,
        "metrics": {
            "a_mm":      [round(v * 1000, 1) for v in a],
            "epsilon":   [round(e[0], 3), round(e[1], 3)],
            "rms_mm":    round(fit["rms"] * 1000, 3),
            "length_mm": length_mm,
            "n_iters":   fit["n_iters"],
        },
        "elapsed_ms": elapsed,
    }
