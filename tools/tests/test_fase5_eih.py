"""Fase 5, niveles 1 y 2: paridad del refinamiento EiH con PozoleV3 sobre datos reales de cam2.

1. Fusión: /atole/perception/fuse_views con los 20 pares PPP de Itabuna_EiH_Fused frente a la nube
   fusionada que guardó PozoleV3.
2. Estimación EiH: para varias vistas reales de cam2 (con la pose del TCP grabada), el pipeline de
   PozoleV3 (máscara → completion ckpt EiH → SQ 800/100 → PCA → signo por el eje previsto) frente a
   /atole/perception/estimate_pods con camera=cam2, eih y la misma referencia y eje previsto.

Uso (AtoleROS lanzado con sim:=true, entorno ROS cargado):
    ~/venvs/main312/bin/python tools/tests/test_fase5_eih.py
"""
import ast
import glob
import json
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from atole_interfaces.action import EstimatePods
from atole_interfaces.srv import FuseViews, LoadDataset
from geometry_msgs.msg import Point, Vector3
from rclpy.action import ActionClient
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/atole_perception'))
from atole_perception.ros_utils import cloud_msg, cloud_to_array, matrix_to_pose  # noqa: E402

POZOLE = Path.home() / 'Documents/pre-Alpha/Atole/PozoleV3'
ROOT = Path.home() / 'Documents/Corvus/datasets'
_EIH = sorted(d.name for d in (ROOT / 'Itabuna_EiH').iterdir() if d.is_dir())
VIEWS = ([f'Itabuna_EiH/{_EIH[i]}' for i in np.linspace(0, len(_EIH) - 1, 6).astype(int)]      # repartidas por el dataset
         + ['Itabuna_plus/20260624_175021/eih03', 'Itabuna_plus/20260624_194134/eih10', 'Itabuna_plus/20260624_183436/eih05'])
N_EXP = np.array([0.0, 0.0, 1.0])          # eje previsto en base_link (pod colgando: de la base a la punta hacia arriba)
sys.path.insert(0, str(POZOLE))
from completion_methods import adapointr as pz_adapointr                              # noqa: E402
from completion_methods.preprocess import preprocess_for_completion as pz_preprocess  # noqa: E402
from devices.inference import MaskRCNNInference                                       # noqa: E402
from pose_estimator import estimate_pose as pz_estimate_pose                           # noqa: E402
from superquadric_pose import fit_and_sample as pz_fit_and_sample                      # noqa: E402


def pozole_functions():
    tree = ast.parse((POZOLE / 'server.py').read_text())
    ns = {'np': np, 'cv2': cv2, 'Optional': Optional}
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', '') in ('PC_MASK_ERODE_PX', 'PC_CONF_THR', 'PC_DEPTH_GRAD_THR')
                                                for t in node.targets):
            exec(compile(ast.Module([node], []), 'server.py', 'exec'), ns)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in ('_align_mask_to_pc', '_refine_object_mask'):
            exec(compile(ast.Module([node], []), 'server.py', 'exec'), ns)
    return ns


def view_files(view):
    p = ROOT / view
    if p.is_dir():
        f = {k: p / n for k, n in (('rgb', 'rgb.png'), ('cloud', 'pointcloud.npy'), ('conf', 'confidence.npy'), ('calib', 'calibration.json'))}
    else:
        f = {k: p.parent / f'cam2_{p.name}_{n}' for k, n in (('rgb', 'rgb.png'), ('cloud', 'pointcloud.npy'),
                                                               ('conf', 'confidence.npy'), ('calib', 'calibration.json'))}
    j = json.loads(f['calib'].read_text())
    T = np.array(j.get('T_base_to_camera_cam2') or j['T_base_to_camera']).reshape(4, 4)
    return f, T


def reference(view, model, ns):
    """PozoleV3 EiH (_eih_extract_mask_points + _eih_correction_estimate_sync, completion_sq) para la
    primera detección válida (≥120 puntos). Devuelve centroide y pose en base_link."""
    f, T = view_files(view)
    pc, conf = np.load(f['cloud']), np.load(f['conf'])
    H, W = pc.shape[:2]
    for det in model.run(cv2.imread(str(f['rgb']))):
        if det.class_name.lower() == 'trunk':
            continue
        m = ns['_refine_object_mask'](ns['_align_mask_to_pc'](det.mask, H, W), conf, pc[:, :, 2])
        xyz = pc[m][:, :3]
        xyz = xyz[np.isfinite(xyz).all(1) & (xyz[:, 2] > 0.05) & (xyz[:, 2] < 20)].astype(np.float32)
        if len(xyz) < 120:
            continue
        pre, _ = pz_preprocess(xyz)
        comp = pz_adapointr.run(pre, {'ckpt': '/home/user/Documents/pre-Alpha/models/EiH_Completion.pth'})
        sq = pz_fit_and_sample(np.asarray(comp['xyz'], np.float32), {'max_pts': 800, 'max_iter': 100})
        r = pz_estimate_pose(sq['surface_xyz'])
        base, tip, axis = r.grasp_position, r.tip_position, r.grasp_normal
        if np.dot(axis, T[:3, :3].T @ N_EXP) < 0:
            base, tip, axis = tip, base, -axis
        to_b = lambda p: T[:3, :3] @ p + T[:3, 3]
        return {'centroid': to_b(xyz.mean(0)), 'grasp': to_b(base), 'tip': to_b(tip), 'axis': T[:3, :3] @ axis,
                'n': len(xyz), 'score': det.score}
    return None


def call(node, client, req, timeout=60):
    f = client.call_async(req)
    rclpy.spin_until_future_complete(node, f, timeout_sec=timeout)
    return f.result()


def main():
    rclpy.init()
    node = rclpy.create_node('fase5_eih_test')
    xyz = lambda p: np.array([p.x, p.y, p.z])

    # ── 1. fusión por el servicio ──
    fuse = node.create_client(FuseViews, '/atole/perception/fuse_views')
    fuse.wait_for_service(timeout_sec=10)
    exact = total = 0
    worst, counts = 0.0, []
    dirs = [d + '/' for d in sorted(glob.glob(str(ROOT / 'Itabuna_EiH_Fused/*')))]
    for d in dirs:
        meta = json.load(open(d + 'eih_fused_meta.json'))
        t2 = np.array(meta['T_base_cam2_view2']).reshape(4, 4)
        to_base = lambda p: np.load(d + p).astype(np.float64) @ t2[:3, :3].T + t2[:3, 3]    # las vistas vienen en el marco de la vista 2
        res = call(node, fuse, FuseViews.Request(clouds=[cloud_msg(to_base('eih_view1_pointcloud.npy'), 'base_link', node.get_clock().now().to_msg()),
                                                         cloud_msg(to_base('eih_view2_pointcloud.npy'), 'base_link', node.get_clock().now().to_msg())],
                                                 target_camera_pose=matrix_to_pose(t2), voxel_m=meta['voxel_m']))
        got, ref = cloud_to_array(res.fused), np.load(d + 'eih_fused_pointcloud.npy')
        dist, _ = cKDTree(got).query(ref)            # como conjunto: la ida y vuelta por base_link en float32
        exact += int((dist < 1e-6).sum())            # puede cambiar de vóxel algún punto y desplazar el orden
        total += len(ref)
        worst = max(worst, float(dist.max()))
        counts.append(len(got) - len(ref))
    print(f'1) fusión por servicio ({len(dirs)} pares PPP): {exact}/{total} puntos de PozoleV3 con gemelo exacto '
          f'({100 * exact / total:.3f} %) · distancia máx al más cercano {worst * 1000:.2f} mm · '
          f'diferencia de tamaño por par: {min(counts)}..{max(counts)}')

    # ── 2. estimación EiH ──
    model = MaskRCNNInference('/home/user/Documents/pre-Alpha/models/ItabunaM_EiH2.pth', 0.8, 'auto', 'itabuna')
    model.load()
    ns = pozole_functions()
    load = node.create_client(LoadDataset, '/atole/sim/load_eih')
    load.wait_for_service(timeout_sec=10)
    est = ActionClient(node, EstimatePods, '/atole/perception/estimate_pods')
    est.wait_for_server(timeout_sec=10)
    rows = []
    for view in VIEWS:
        ref = reference(view, model, ns)
        if ref is None:
            print(f'   {view}: sin detección válida en PozoleV3')
            continue
        print('   ', call(node, load, LoadDataset.Request(folder=view, rate_hz=2.0)).message)
        time.sleep(1.5)
        goal = EstimatePods.Goal(camera='cam2', eih=True, gate_m=0.05, reference=Point(**dict(zip('xyz', map(float, ref['centroid'])))),
                                 expected_axis=Vector3(**dict(zip('xyz', map(float, N_EXP)))))
        gf = est.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, gf)
        rf = gf.result().get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=120)
        pods = [p for p in rf.result().result.pods.pods if p.status == 'ok']
        if not pods:
            print(f'   {view}: AtoleROS no asoció el pod ({[p.status for p in rf.result().result.pods.pods]})')
            continue
        p = pods[0]
        dg = 1000 * np.linalg.norm(ref['grasp'] - xyz(p.grasp_bottom))
        dt = 1000 * np.linalg.norm(ref['tip'] - xyz(p.tip))
        da = np.degrees(np.arccos(np.clip(np.dot(ref['axis'], xyz(p.axis)), -1, 1)))
        rows.append((dg, dt, da))
        print(f'   {view}: pts {ref["n"]}/{p.n_partial} · Δgrasp {dg:.2f} mm · Δpunta {dt:.2f} mm · Δeje {da:.2f}°')
    if rows:
        r = np.array(rows)
        print(f'2) estimación EiH en {len(rows)} vistas: Δgrasp máx {r[:, 0].max():.2f} mm · Δpunta máx {r[:, 1].max():.2f} mm · '
              f'Δeje máx {r[:, 2].max():.2f}°')
    pz_adapointr.shutdown_worker()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
