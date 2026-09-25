# Refinamiento EiH de PozoleV3 (referencia para la Fase 5)

Fuente: ~/Documents/pre-Alpha/Atole/PozoleV3/server.py (líneas indicadas). Los tres botones son
comandos WS manuales; el paso EIH_REFINE de harvest_sequence estaba desactivado (5602-5617).

## Común
- Parte siempre de la pose EtH (grasp_position_robot = extremo base, grasp_normal_robot, quaternion_robot).
- Captura (_eih_capture_select 1961): 1 grab tras settle 0.4 s; TCP leído después del grab.
- T_base←cam2 = _tcp_to_T(tcp) @ inv(O_live) @ O_ref(=I) @ X  (1337) = brida @ X (X = T_tcp_to_camera_cam2).
  _tcp_to_T AUBO: Rotation.from_euler('ZYX', [rz, ry, rx]).
- Máscara → puntos (1342): mismos filtros que EtH (erosión 9×9, conf<50, grad<0.03, 0.05<Z<20). Modelo EiH (score 0.80), excluye "trunk".
- Asociación (_eih_pick_detection_correlated 1408): centroide de los puntos crudos (≥120) en base, distancia a la
  referencia (grasp EtH o el corregido); el más cercano; si > 0.20 m → error "pod equivocado".
- Estimación (1858, completion_sq): preproceso + AdaPoinTr ckpt EiH → SQ (800/100) → PCA; si eje·n_esperado<0 se
  invierte el eje y se intercambian base/punta; centroid=(base+punta)/2; pick_bottom = base.
- Actualización (_eih_build_update 1574): g_new = T·base (reemplazo, sin mezcla); n_new = R·eje;
  R_delta = align(n0 → n_new) limitado a 15°; q_new = R_delta·q0 (solo la orientación usa el límite).
- Movimiento (_eih_apply_correction_move 2044): pre = g_new − n_eff·ApproachDist; se mueve si > 4 mm (moveL).

## SINGLE (_eih_run_once 2159): settle, captura, estima, un movimiento corregido.
## FUSED_2VIEW (2388): vista 1 → movimiento corregido (o +0.03 m en Y base si no se movió) → settle →
   vista 2 (referencia = grasp corregido) → fusión en el marco de cam2 de la vista 2 → estimación → movimiento.
## PPP (2704): pos_pp = pre-pick EtH; home2_xy = FK(Home2).xy; d = unit(pp.xy − home2.xy);
   PPP.xy = pp.xy − 0.10·d, z = pp.z; orientación: 6 iteraciones apuntando el eje óptico empírico al grasp C,
   inclinación limitada a 35° respecto a la del pre-pick. Mover PPP → settle → vista 1 → mover a pre-pick →
   settle → vista 2 → fusión (marco vista 2) → guarda Itabuna_EiH_Fused → estimación → movimiento (gate 0.20 m
   del punto medio fusionado a g0).
## Fusión (_voxel_merge 2360): P = vstack(P1 en marco c2, P2) float64; keys = floor(P/0.003);
   np.unique(keys, axis=0, return_index=True) → P[sort(idx)]. Reproduce bit a bit Itabuna_EiH_Fused (20/20).
## Datasets: Itabuna_EiH (PozoleV2 SnapEiH) e Itabuna_plus/*/cam2_eihNN: poses manuales arbitrarias,
   T_base_to_camera = TCP @ X (sin anclaje a la brida).
## Constantes: CorrelateGate 0.20, min 120 pts, move 4 mm, settle 0.4, nudge 0.03, PPP 0.10/35°, voxel 0.003.
