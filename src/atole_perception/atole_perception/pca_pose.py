"""Portado sin cambios de PozoleV3 (pose_estimator.py, sin el test sintético): mismo algoritmo.


pose_estimator.py
=================
Estimación de pose 6D de un pod/fruto colgante a partir de una nube de puntos
segmentada. No depende de ningún módulo del proyecto — solo numpy, open3d y scipy.

Coordenadas de entrada y salida: frame ZED
    X = derecha, Y = abajo, Z = hacia adelante (forward)
    Unidades: metros

Uso:
    from pose_estimator import estimate_pose, PoseResult
    result = estimate_pose(points_xyz)   # points_xyz: (N, 3) float32/64
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
#  Filtros robustos (pre-PCA)
# ─────────────────────────────────────────────────────────────────────────────

def _mad_filter(pts: np.ndarray, k_sigma: float = 3.0) -> np.ndarray:
    """Devuelve máscara booleana: puntos dentro de k_sigma·MAD del centroide.
    Elimina stragglers que sobreviven al DBSCAN (clusters pequeños cercanos,
    puntos dispersos) y que tirarían el PCA."""
    centroid = np.mean(pts, axis=0)
    dists    = np.linalg.norm(pts - centroid, axis=1)
    med      = float(np.median(dists))
    mad      = float(np.median(np.abs(dists - med)))
    if mad < 1e-6:
        return np.ones(len(pts), dtype=bool)
    # 1.4826 convierte MAD a σ-equivalente bajo normal
    threshold = med + k_sigma * 1.4826 * mad
    return dists < threshold


def _ransac_line(pts: np.ndarray, inlier_dist: float = 0.07,
                 n_iter: int = 100, min_inliers: int = 20,
                 seed: int = 42) -> tuple:
    """RANSAC line fit en 3D. Muestrea 2 puntos, define línea, cuenta inliers
    con distancia perpendicular < inlier_dist. Retorna (axis, centroid, inlier_mask, sv, Vt)
    donde axis sale de un PCA final solo con los inliers (más estable que
    el par muestreado). Vt es la matriz completa de PCs (3×3): Vt[0]=axis,
    Vt[1]=ancho, Vt[2]=profundidad."""
    rng = np.random.default_rng(seed)
    n = len(pts)
    if n < min_inliers:
        return None, None, None, None, None

    best_mask  = None
    best_count = 0

    for _ in range(n_iter):
        i, j = rng.choice(n, 2, replace=False)
        d = pts[j] - pts[i]
        d_norm = np.linalg.norm(d)
        if d_norm < 1e-6:
            continue
        d = d / d_norm
        v = pts - pts[i]
        v_dot_d = v @ d
        # ||v||^2 - (v·d)^2 = d_perp^2
        dist_sq = np.sum(v * v, axis=1) - v_dot_d * v_dot_d
        mask = dist_sq < inlier_dist * inlier_dist
        cnt  = int(mask.sum())
        if cnt > best_count:
            best_count = cnt
            best_mask  = mask

    if best_mask is None or best_count < min_inliers:
        return None, None, None, None, None

    inlier_pts = pts[best_mask]
    centroid   = np.mean(inlier_pts, axis=0)
    _, sv, Vt  = np.linalg.svd(inlier_pts - centroid, full_matrices=False)
    axis = Vt[0]
    return axis, centroid, best_mask, sv, Vt


# ─────────────────────────────────────────────────────────────────────────────
#  Resultado público
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PoseResult:
    success:        bool
    position:       np.ndarray   # (3,) centroide del pod, frame ZED (X right, Y down, Z fwd), metros
    quaternion:     np.ndarray   # (4,) xyzw, orientación del pod en frame ZED
    axis:           np.ndarray   # (3,) eje longitudinal del pod (unit vector), frame ZED
    grasp_position: np.ndarray   # (3,) punto de agarre base convexa (abajo), frame ZED
    grasp_normal:   np.ndarray   # (3,) normal de agarre apuntando hacia la cámara, frame ZED
    tip_position:   np.ndarray   # (3,) extremo superior / rama (arriba), frame ZED
    confidence:     float        # 0.0 a 1.0
    elapsed_ms:     float
    # Ancho / offset de profundidad
    # width_m         = diámetro del pod medido a 90° del eje longitudinal (proyección sobre PC2)
    # offset_deep_m   = width_m / 2 — compensa que la cámara solo ve la superficie frontal
    # offset_direction= eje PC3 (perpendicular a eje y a ancho), frame ZED, apunta hacia +Z (lejos de la cámara)
    width_m:          float      = 0.0
    offset_deep_m:    float      = 0.0
    offset_direction: np.ndarray = field(default_factory=lambda: np.array([0., 0., 1.]))
    error_msg:        str = ""


def _zero_result(elapsed_ms: float, error_msg: str) -> PoseResult:
    z3 = np.zeros(3, dtype=np.float64)
    return PoseResult(
        success=False,
        position=z3.copy(), quaternion=np.array([0., 0., 0., 1.]),
        axis=z3.copy(), grasp_position=z3.copy(), grasp_normal=z3.copy(),
        tip_position=z3.copy(),
        confidence=0.0, elapsed_ms=elapsed_ms, error_msg=error_msg,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Estimador principal
# ─────────────────────────────────────────────────────────────────────────────

def estimate_pose(points: np.ndarray) -> PoseResult:
    """
    Entrada : nube de puntos segmentada, shape (N, 3), float32/float64
              coordenadas en frame ZED (X=right, Y=down, Z=forward), metros
              Ya viene segmentada — solo el pod, sin fondo
    Salida  : PoseResult completo
    """
    t0 = time.perf_counter()

    try:
        import open3d as o3d
        from scipy.spatial.transform import Rotation

        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            return _zero_result(0.0, f"Invalid input shape: {points.shape}")

        # ── Paso 1 — Voxel downsample (defensivo: si no pasó por _apply_filters,
        # esto uniformiza la densidad para que PCA/RANSAC no se sesguen por
        # zonas sobrerrepresentadas del sensor). Se quita el SOR porque ya se
        # aplica upstream en server._apply_filters.
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size=0.004)

        pts = np.asarray(pcd.points, dtype=np.float64)
        n_points_clean = len(pts)

        if n_points_clean < 50:
            elapsed = (time.perf_counter() - t0) * 1000
            return _zero_result(elapsed, f"Too few points after cleaning: {n_points_clean}")

        # ── Paso 2 — MAD filter (k=3σ-equivalente) ────────────────────────────
        # Pod de cacao: 10-20cm largo, 7-12cm ancho → diagonal máxima ~12cm.
        # MAD elimina puntos alejados del centroide antes del RANSAC.
        mad_mask = _mad_filter(pts, k_sigma=3.0)
        pts = pts[mad_mask]
        n_after_mad = len(pts)
        if n_after_mad < 50:
            elapsed = (time.perf_counter() - t0) * 1000
            return _zero_result(elapsed, f"Too few points after MAD: {n_after_mad}")

        # ── Paso 3 — RANSAC line fit para el eje ─────────────────────────────
        # inlier_dist = 0.07m: cubre radio máximo del pod (~6cm) + margen de
        # ruido de profundidad del ZED (~1cm). Puntos más lejos de la línea
        # central son ruido y NO participan del PCA final.
        axis, centroid_in, inlier_mask, singular_values, Vt_full = _ransac_line(
            pts, inlier_dist=0.07, n_iter=150, min_inliers=30
        )
        if axis is None:
            # Fallback a PCA plano sobre todos los puntos filtrados por MAD
            position = np.mean(pts, axis=0)
            _, singular_values, Vt_full = np.linalg.svd(pts - position, full_matrices=False)
            axis = Vt_full[0]
            inlier_mask = np.ones(len(pts), dtype=bool)
            n_inliers = len(pts)
        else:
            pts = pts[inlier_mask]
            position = centroid_in
            n_inliers = len(pts)

        # Resolver ambigüedad de dirección:
        # En frame ZED, Y apunta hacia abajo. El pod cuelga — la base (donde se agarra)
        # está más abajo (Y más alto). Orientar axis para que apunte hacia Y positivo.
        pts_centered = pts - position
        projections = pts_centered @ axis
        idx_top    = projections < np.percentile(projections, 15)   # extremo rama (Y menor)
        idx_base   = projections > np.percentile(projections, 85)   # extremo base (Y mayor)
        y_top  = np.mean(pts[idx_top,  1]) if idx_top.any()  else position[1] - 0.1
        y_base = np.mean(pts[idx_base, 1]) if idx_base.any() else position[1] + 0.1

        if y_base < y_top:
            # axis apunta hacia arriba (Y menor), invertir para que apunte a la base
            axis = -axis

        # Quaternion: se calcula después de obtener grasp_normal (base→tip = arriba)
        # para que Z del TCP apunte hacia arriba a lo largo del pod
        quaternion = None  # placeholder, se calcula en paso 4

        # ── Paso 4 — Grasp point (base convexa) + Tip (rama) ───────────────
        # Recalcular proyecciones con el eje ya orientado (post-inversión)
        projections = pts_centered @ axis

        # Base = percentil alto (dirección del eje = hacia abajo)
        base_threshold = np.percentile(projections, 85)
        base_mask = projections > base_threshold
        base_pts  = pts[base_mask]
        grasp_position = np.mean(base_pts, axis=0)

        # Tip = percentil bajo (extremo opuesto = rama / arriba)
        tip_threshold = np.percentile(projections, 15)
        tip_mask = projections < tip_threshold
        tip_pts  = pts[tip_mask]
        tip_position = np.mean(tip_pts, axis=0)

        # Dirección de agarre: vector unitario de grasp (base) → tip (rama)
        grasp_vec = tip_position - grasp_position
        grasp_len = np.linalg.norm(grasp_vec)
        grasp_normal = grasp_vec / grasp_len if grasp_len > 1e-6 else np.array([0., 0., -1.])

        # Quaternion que rota [0, 0, 1] → grasp_normal (TCP Z apunta hacia arriba = tip)
        ref = np.array([0., 0., 1.])
        cross = np.cross(ref, grasp_normal)
        cross_norm = np.linalg.norm(cross)
        if cross_norm < 1e-6:
            if np.dot(ref, grasp_normal) > 0:
                quaternion = np.array([0., 0., 0., 1.])
            else:
                quaternion = np.array([0., 1., 0., 0.])
        else:
            rot, _ = Rotation.align_vectors([grasp_normal], [ref])
            quaternion = rot.as_quat()  # xyzw

        # ── Paso 4b — Ancho (diámetro) y offset de profundidad ────────────────
        # PC2 = eje perpendicular al eje del pod con mayor varianza = ancho.
        # PC3 = perpendicular a PC1 y PC2 = dirección de profundidad.
        # width_m = extensión del pod proyectado sobre PC2 (diámetro).
        # offset_deep_m = width_m / 2 — compensa el sesgo del centroide: la
        # cámara solo ve la cara frontal, el centroide real está "más al fondo"
        # una distancia ≈ radio.
        pc2 = Vt_full[1]
        pc3 = Vt_full[2]
        proj_pc2 = pts_centered @ pc2
        width_m   = float(np.percentile(proj_pc2, 97.5) - np.percentile(proj_pc2, 2.5))
        offset_deep_m = width_m / 2.0
        # PC3 apunta hacia +Z (lejos de la cámara = hacia el interior del pod).
        # En frame ZED, Z+ = forward. Si PC3 tiene componente Z negativa, invertir.
        if pc3[2] < 0:
            pc3 = -pc3
        offset_direction = pc3

        # ── Paso 5 — Confidence ──────────────────────────────────────────────
        # inlier_ratio penaliza cuando RANSAC tuvo que descartar mucho (PC ruidoso
        # con muchos outliers = señal débil). pca_ratio mide cuán elongada es la
        # forma: ratio alto = pod claramente longitudinal.
        n_score      = min(1.0, n_inliers / 500.0)
        inlier_ratio = n_inliers / max(1, n_after_mad)
        pca_ratio    = singular_values[0] / (singular_values[1] + 1e-6)
        axis_score   = min(1.0, pca_ratio / 3.0)
        confidence   = n_score * axis_score * inlier_ratio

        elapsed_ms = (time.perf_counter() - t0) * 1000

        return PoseResult(
            success=True,
            position=position,
            quaternion=quaternion,
            axis=axis,
            grasp_position=grasp_position,
            grasp_normal=grasp_normal,
            tip_position=tip_position,
            confidence=float(confidence),
            elapsed_ms=elapsed_ms,
            width_m=float(width_m),
            offset_deep_m=float(offset_deep_m),
            offset_direction=offset_direction,
        )

    except Exception as e:
        elapsed_ms = (time.perf_counter() - t0) * 1000
        return _zero_result(elapsed_ms, str(e))
