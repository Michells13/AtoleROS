"""Máscara de Mask R-CNN → nube parcial del pod en el marco óptico de la cámara.

Mismos filtros que PozoleV3 (_align_mask_to_pc, _refine_object_mask, _extract_pc_frozen):
  1. Píxel válido: Z finita y 0.05 < Z < 20 m.
  2. Erosión de la máscara (kernel k×k, k = max(3, 2·erode_px + 1), 1 iteración).
  3. Confianza ZED < conf_thr (0 = mejor, 100 = peor; NaN/inf = 100).
  4. Gradiente morfológico 3×3 de Z < grad_thr (descarta bordes y flying pixels).
  5. Submuestreo por stride uniforme hasta max_points.
PozoleV3 leía X, Y de la nube organizada de la ZED; aquí se reconstruyen desde la profundidad y
los intrínsecos rectificados (X = (u − cx)·Z/fx), que dan la misma nube (< 0.003 mm).
"""
import cv2
import numpy as np

Z_MIN, Z_MAX = 0.05, 20.0


def align_mask(mask, h, w):
    """Ajusta una máscara de la imagen RGB al tamaño de la profundidad (escala + recorte arriba)."""
    mh, mw = mask.shape[:2]
    if (mh, mw) == (h, w):
        return mask
    if mw == w:
        out = np.zeros((h, w), np.uint8)
        out[:min(mh, h)] = mask[:min(mh, h)]
        return out
    new_h = int(round(mh * w / mw))
    scaled = cv2.resize(mask, (w, new_h), interpolation=cv2.INTER_NEAREST)
    out = np.zeros((h, w), np.uint8)
    out[:min(new_h, h)] = scaled[:min(new_h, h)]
    return out


def refine_mask(mask, conf, z, erode_px=4, conf_thr=50.0, grad_thr=0.03):
    """Máscara booleana con solo los píxeles fiables (mismo orden y parámetros que PozoleV3)."""
    m = mask if mask.dtype == np.uint8 else mask.astype(np.uint8)
    if erode_px and erode_px > 0:
        k = max(3, 2 * int(erode_px) + 1)
        m = cv2.erode(m, np.ones((k, k), np.uint8), iterations=1)
    refined = m > 0
    if conf_thr is not None and conf is not None and conf.shape[:2] == refined.shape:
        c = np.nan_to_num(conf.astype(np.float32), nan=100.0, posinf=100.0, neginf=100.0)
        refined &= c < conf_thr
    if grad_thr is not None and z is not None and z.shape[:2] == refined.shape:
        zf = z.astype(np.float32)
        valid = np.isfinite(zf) & (zf > Z_MIN) & (zf < Z_MAX)
        z_safe = np.where(valid, zf, 1e6).astype(np.float32)
        grad = cv2.morphologyEx(z_safe, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        refined &= (grad < grad_thr) & valid
    return refined


def object_points(depth, k, refined, max_points=20000):
    """(N, 3) float32 en el marco óptico (X derecha, Y abajo, Z adelante), en metros."""
    h, w = depth.shape
    z = depth.ravel()
    valid = np.isfinite(z) & (z > Z_MIN) & (z < Z_MAX)
    idx = np.flatnonzero(valid & refined.ravel())
    if len(idx) > max_points:
        idx = idx[::len(idx) // max_points][:max_points]
    v, u = np.divmod(idx, w)
    zs = z[idx].astype(np.float64)
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    return np.stack([(u - cx) * zs / fx, (v - cy) * zs / fy, zs], axis=1).astype(np.float32)


def full_mask(det_mask, mask_xy, h, w):
    """Reconstruye la máscara a tamaño imagen desde su recorte (Detection.mask + mask_xy)."""
    out = np.zeros((h, w), np.uint8)
    x, y = mask_xy
    mh, mw = det_mask.shape
    out[y:y + mh, x:x + mw] = det_mask[:max(0, min(mh, h - y)), :max(0, min(mw, w - x))]
    return out
