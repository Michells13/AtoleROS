# Calibración hand-eye de PozoleV3 (referencia para la Fase 7)

Fuente: ~/Documents/pre-Alpha/Atole/PozoleV3/server.py (líneas indicadas). Mapa hecho por lectura del
código; los resultados numéricos marcados como preliminares se verificarán en la Fase 7.

## Captura (_calib_run_sequence 8733)
- Poses por cámara en Config.xml `<Calibration><Poses cam="camN">` (joints + TCP), 21/21/20 ahora.
- Por pose: moveJ (speed 0.15, accel 0.1), asentar 0.8 s, TCP fresco, grab, detección y (opcional)
  guardar `calibImages/<cam>/pose_NNN/{image.png, tcp.txt}` + `intrinsics.json` una vez por carpeta.
- Imagen: izquierda rectificada (BGR). Intrínsecos: left_cam de la ZED, dist = 0.
- tcp.txt: x y z rx ry rz (AUBO getTcpPose, incluye el offset de herramienta); convención AUBO
  `Rotation.from_euler('ZYX', [rz, ry, rx])` (_tcp_to_T 8592).

## Detección (_detect_charuco 8534)
- Tablero de Config.xml: 8×8, cuadro 0.0254 m, marcador 0.01905 m, DICT_4X4_50, patrón nuevo
  (sin setLegacyPattern). `cv2.aruco.CharucoDetector(board).detectBoard(gray)` con parámetros por defecto.
- Mínimo 6 esquinas (fijo; el "Min corners" de la GUI no se usa).
- `board.matchImagePoints` → `cv2.solvePnP` (ITERATIVE) → rvec/tvec = cámara ← tablero.
- Requiere OpenCV ≥ 4.7 (CharucoDetector): el venv main312 tiene 4.13; el python del sistema 4.6 no sirve.

## Cálculo (_task_calib_from_images 8134, offline; _calib_compute 8843, online)
- Carpetas en orden `sorted()`; mínimo 3 pares válidos.
- EtH (cam0/cam1): A_i = inv(T_tcp_i · T_tool) como gripper2base, B_i = (R(rvec), tvec) como
  target2cam; `cv2.calibrateHandEye(..., method)`; el resultado es base ← cámara, sin invertir.
  T_tool = T_tool_to_tcp de Config.xml (se cancela en AX = XB).
- EiH (cam2): A_i = T_tcp_i tal cual; el resultado es TCP ← cámara.
- Métodos: TSAI (por defecto), PARK, ANDREFF; cualquier otro cae a TSAI.
- Métrica de consistencia (solo online): EtH off_i = tvec_i − (R·tcp_i + t); EiH tablero en base
  p_i = R_tcp_i·(R·tvec_i + t) + t_tcp_i; res_i = ‖off_i − media‖; aviso si > 0.02 m (no se descarta).
  Avisos: |t| fuera de rango y ángulo EtH > 30° respecto a [[0,0,−1],[1,0,0],[0,−1,0]].

## Guardado (calib_save_result 6254)
- cam0 → T_robot_to_camera_cam0, cam1 → T_robot_to_camera, cam2 → T_tcp_to_camera_cam2; recarga el TF.

## Datasets en disco
- `Atole/PozoleV3/calibImages/{cam0,cam1,cam2}`: 21 poses cada una (2026-06-24), image.png + tcp.txt +
  intrinsics.json. Todas detectan el tablero (19–49 esquinas).
- `Itabuna3D/double/calibration/{cam0,cam1}`: robot UR (rotvec), rig anterior; no aplica al AUBO.
- `ros_calibration_trajectories_1/.../datasets/cam{0,1}_pozolev3`: los mismos datos en formato
  industrial_calibration (para el backend Tecnalia).

## Resultados preliminares (reproducción del agente, a verificar en la Fase 7)
- cam0 y cam1: las matrices de Config.xml se reproducen exactamente con el dataset (0.00 mm).
- **cam2: la matriz en uso NO sale de calibImages/cam2**: el recálculo da t ≈ (0.126, −0.027, 0.018)
  frente a (0.045, −0.024, 0.110) en Config.xml (123 mm y 0.68° de diferencia). La matriz en uso ya
  existía antes de capturar ese dataset (Config_preEyeToHand_20260616). Posible causa: otro offset de
  TCP en el controlador al capturar (sin comprobar). Es la matriz que dio buena fusión cam1 + cam2.
