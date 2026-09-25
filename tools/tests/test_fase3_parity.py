"""Paridad de la percepción EtH de AtoleROS con PozoleV3 sobre el mismo dataset.

Referencia = el código ORIGINAL de PozoleV3 (Mask R-CNN de devices/inference.py, _align_mask_to_pc
y _refine_object_mask extraídas de server.py, preprocess + worker AdaPoinTr, superquadric_pose,
pose_estimator) sobre cam1_*_pointcloud.npy, pasado a base con T_robot_to_camera de su Config.xml.
AtoleROS = la acción /atole/perception/estimate_pods con el sistema en SIM (sim:=true).

Uso (con AtoleROS lanzado en SIM y el entorno ROS cargado):
    ~/venvs/main312/bin/python tools/tests/test_fase3_parity.py [carpeta ...]
"""
import ast
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from atole_interfaces.action import EstimatePods
from atole_interfaces.srv import LoadDataset
from rclpy.action import ActionClient

POZOLE = Path.home() / 'Documents/pre-Alpha/Atole/PozoleV3'
DATASET = Path.home() / 'Documents/Corvus/datasets/Itabuna_plus'
FOLDERS = sys.argv[1:] or ['20260422_144848_954', '20260422_150722_628', '20260422_144209_778']
sys.path.insert(0, str(POZOLE))

from completion_methods import adapointr as pz_adapointr          # noqa: E402
from completion_methods.preprocess import preprocess_for_completion as pz_preprocess  # noqa: E402
from devices.inference import MaskRCNNInference                   # noqa: E402
from pose_estimator import estimate_pose as pz_estimate_pose        # noqa: E402
from superquadric_pose import fit_and_sample as pz_fit_and_sample   # noqa: E402


def pozole_functions():
    """_align_mask_to_pc y _refine_object_mask tal cual están en server.py (sin importar el servidor)."""
    src = (POZOLE / 'server.py').read_text()
    tree = ast.parse(src)
    ns = {'np': np, 'cv2': cv2, 'Optional': Optional}
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', '') in ('PC_MASK_ERODE_PX', 'PC_CONF_THR', 'PC_DEPTH_GRAD_THR')
                                                for t in node.targets):
            exec(compile(ast.Module([node], []), 'server.py', 'exec'), ns)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in ('_align_mask_to_pc', '_refine_object_mask'):
            exec(compile(ast.Module([node], []), 'server.py', 'exec'), ns)
    return ns


def pozole_T():
    root = ET.parse(POZOLE / 'Config.xml').getroot()
    el = next(e for e in root.iter('Transform') if e.get('name') == 'T_robot_to_camera')
    vals = [float(v) for v in ' '.join(el.itertext()).split()]
    return np.array(vals[:16]).reshape(4, 4)


def reference(folder, model, ns, T):
    """Pipeline de PozoleV3 (botón superquadric: completion + SQ + PCA) para todas las detecciones."""
    d = DATASET / folder
    img = cv2.imread(str(next(p for p in d.glob('cam1_*_rgb.png') if '_right' not in p.stem)))
    pc = np.load(next(d.glob('cam1_*_pointcloud.npy')))
    conf_file = next(d.glob('cam1_*_confidence.npy'), None)
    conf = np.load(conf_file) if conf_file else None                  # los datasets no traen confianza
    H, W = pc.shape[:2]
    X, Y, Z = (pc[:, :, i].ravel() for i in range(3))
    valid = np.isfinite(Z) & (Z > 0.05) & (Z < 20.0)
    out = []
    for det in model.run(img):
        mask = ns['_refine_object_mask'](ns['_align_mask_to_pc'](det.mask, H, W), conf, pc[:, :, 2])
        idx = np.where(valid & mask.ravel())[0]
        obj_max = max(5000, 80000 // 4)
        if len(idx) > obj_max:
            idx = idx[::len(idx) // obj_max][:obj_max]
        pts = np.stack([X[idx], Y[idx], Z[idx]], 1).astype(np.float32)
        r = {'score': det.score, 'n_partial': len(pts)}
        pre, _ = pz_preprocess(pts)
        comp = pz_adapointr.run(pre)
        if comp['success']:
            dense = np.asarray(comp['xyz'], np.float32)
            sq = pz_fit_and_sample(dense)
            pose = pz_estimate_pose(sq['surface_xyz']) if sq['success'] else None
            if pose is not None and pose.success:
                to_b = lambda p: T[:3, :3] @ p + T[:3, 3]
                r.update(grasp=to_b(pose.grasp_position), tip=to_b(pose.tip_position), axis=T[:3, :3] @ pose.grasp_normal,
                         width=pose.width_m, conf=pose.confidence, length=sq['metrics']['length_mm'] / 1000.0)
        out.append(r)
    return out


def atole(node, folder):
    load = node.create_client(LoadDataset, '/atole/sim/load')
    load.wait_for_service(timeout_sec=10)
    f = load.call_async(LoadDataset.Request(folder=folder, rate_hz=2.0))
    rclpy.spin_until_future_complete(node, f, timeout_sec=30)
    assert f.result().ok, f.result().message
    time.sleep(1.5)                         # que el frame nuevo reemplace al anterior
    client = ActionClient(node, EstimatePods, '/atole/perception/estimate_pods')
    client.wait_for_server(timeout_sec=10)
    gf = client.send_goal_async(EstimatePods.Goal(camera='cam1'))
    rclpy.spin_until_future_complete(node, gf)
    rf = gf.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=300)
    res = rf.result().result
    assert res.ok, res.message
    return res.pods.pods


def main():
    rclpy.init()
    node = rclpy.create_node('fase3_parity')
    model = MaskRCNNInference(str(Path.home() / 'Documents/Corvus/models/ItabunaM.pth'), 0.5, 'auto', 'itabuna')
    model.load()
    ns, T = pozole_functions(), pozole_T()
    xyz = lambda p: np.array([p.x, p.y, p.z])
    worst = {'score': 0.0, 'n_partial': 0, 'grasp_mm': 0.0, 'tip_mm': 0.0, 'axis_deg': 0.0, 'width_mm': 0.0}
    n_cmp = 0
    for folder in FOLDERS:
        ref, got = reference(folder, model, ns, T), atole(node, folder)
        print(f'\n{folder}: PozoleV3 {len(ref)} detecciones · AtoleROS {len(got)} ({sum(p.status == "ok" for p in got)} con pose)')
        assert len(ref) == len(got), 'distinto número de detecciones'
        for r, p in zip(ref, got):
            d_score, d_n = abs(r['score'] - p.score), abs(r['n_partial'] - p.n_partial)
            worst['score'], worst['n_partial'] = max(worst['score'], d_score), max(worst['n_partial'], d_n)
            line = f'  pod {p.id:2d} [{p.status[:16]:16s}] Δscore {d_score:.1e} · parcial {r["n_partial"]}/{p.n_partial}'
            if p.status == 'ok' and 'grasp' in r:
                dg = 1000 * np.linalg.norm(r['grasp'] - xyz(p.grasp_bottom))
                dt = 1000 * np.linalg.norm(r['tip'] - xyz(p.tip))
                da = np.degrees(np.arccos(np.clip(np.dot(r['axis'], xyz(p.axis)), -1, 1)))
                dw = 1000 * abs(r['width'] - p.width_m)
                for k, v in (('grasp_mm', dg), ('tip_mm', dt), ('axis_deg', da), ('width_mm', dw)):
                    worst[k] = max(worst[k], v)
                n_cmp += 1
                line += f' · Δgrasp {dg:.2f} mm · Δtip {dt:.2f} mm · Δeje {da:.2f}° · Δancho {dw:.2f} mm'
            print(line)
    print(f'\nPEOR CASO en {n_cmp} pods con pose: ' + ' · '.join(f'{k} {v:.3g}' for k, v in worst.items()))
    pz_adapointr.shutdown_worker()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
