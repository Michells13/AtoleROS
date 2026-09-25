# AtoleROS — contexto y estado (2026-09-26)

Documento para revisar el proyecto fuera de la ZED Box: qué es, cómo está hecho, qué funciona
(verificado), qué falta y qué hay que saber antes de tocarlo.

## Qué es

Sistema ROS 2 Jazzy de cosecha de mazorcas de cacao para la **ZED Box Duo** (Jetson Orin NX 16 GB,
JetPack 7.2, Ubuntu 24.04), diseñado desde cero con **PozoleV3** (FastAPI monolítico) como guía.
Pipeline automático: Home2 → detección (Mask R-CNN) → filtro de área de trabajo → completion
(AdaPoinTr) → superquadric → PCA → selección con IK → refinamiento eye-in-hand → pre-pick → pick →
cerrar → girar → retirar → Release → Home2.

Hardware:
- Robot **AUBO iS10** (VM en 192.168.100.27 ahora; el real, 192.168.43.100, en ~2 semanas).
- Gripper **RGI-100** (Modbus RTU; 1000 = abierto, 0 = cerrado).
- Cámaras **ZED X HDR** EtH en la base del rover: cam0 (izquierda), cam1 (derecha, SN 64813059),
  camC (centro); solo una activa a la vez. **cam2** ZED X Mini (SN 55177207) eye-in-hand en el TCP.

## Arrancar

```bash
cd ~/Documents/pre-Alpha/AtoleROS
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install   # solo tras cambios
source install/setup.bash
ros2 launch atole_bringup atole.launch.py robot:=vm sim:=true gui:=true
# robot:=mock|vm|real · sim:=true (datasets) | false (cámaras reales) · gui:=true (web GUI)
```

Web GUI: `http://<ZED Box>:8080` (en la ZED Box: `http://localhost:8080`). Llega a READY en ~20 s
(warm-up). Cerrar con Ctrl+C y esperar a que termine. **Nunca lanzar dos pilas a la vez** (una ocupa
~11.7 GB de 15.6; dos colgaron la ZED Box el 2026-09-25).

Para la CLI de ROS contra el sistema:
```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/install/atole_bringup/share/atole_bringup/config/fastdds_nonblocking.xml
```

Lichtblick (visor 3D de la GUI) no está en el repo (170 MB): descargar `lichtblick-web.tar.gz` de la
release v1.29.1 de lichtblick-suite y descomprimirlo en `~/.local/share/atole/lichtblick-1.29.1`
(`Gui/LichtblickDir` en Config.xml).

## Arquitectura

| Paquete | Nodos | Python | Papel |
|---|---|---|---|
| atole_interfaces | — | — | 13 msg, 20 srv, 7 action |
| atole_common | — | — | ConfigView, QoS, `grasp.py` (pre-pick/pick), `eih.py` (corrección EiH, PPP), `static_tf.py`, `sync_calls.py` |
| atole_config | config_manager | sistema | único dueño de `config/Config.xml` |
| atole_system | system_monitor | sistema | salud, warm-up, READY |
| atole_description | robot_state_publisher | — | URDF AUBO iS10 + gripper |
| atole_arm | arm_driver | venv | AUBO: estado 20 Hz, lease, moveJ/moveL cancelables, IK de frente, J6 bloqueado |
| atole_gripper | grip_twist_controller | venv | RGI-100 |
| atole_cameras | camera_manager, extrinsics_publisher | sistema | ZED (un contenedor por cámara), TF de calibración |
| atole_sim | dataset_player | venv | datasets EtH y vistas EiH como si fueran las cámaras |
| atole_perception | detector_node, pod_pose_node, cloud_fusion_node | venv | Mask R-CNN; máscara→nube→completion→SQ→PCA; fusión |
| atole_task_planning | pod_selector | venv | estrategias + alcanzabilidad por IK |
| atole_mission | mission_manager | venv | secuencia, modos todos/siguiente/paso a paso, EiH |
| atole_calibration | calibration_node | venv | **esqueleto (Fase 7)** |
| atole_gui | gui_server, gui_bridge | sistema / venv | web GUI :8080, miniaturas, nubes, overlay |
| atole_bringup | — | — | launch, perfil Fast DDS |

Nodos "venv" = `~/venvs/main312` (torch 2.14, numpy 2). AdaPoinTr corre en `~/venvs/adapointr`
como worker aparte. Grafo interactivo de la arquitectura: https://claude.ai/artifact/LWy6wXZ8wXN6hzgK11eD3j

## Estado por fase

| Fase | Commit | Verificado |
|---|---|---|
| 0 Esqueleto | 1f01918 | 15 paquetes, interfaces, config_manager |
| 1 Robot y gripper | 3608dfd | 15/15 pruebas contra la VM (`tools/tests/test_fase1_arm.py`); RGI-100 real |
| 2 Cámaras, TF, SIM | f5a9c5e | cam1 + cam2 en vivo; TF exacto de la calibración; SIM |
| 3 Percepción | 04542a8 | Mask R-CNN idéntico bit a bit a PozoleV3; pose ≤ 1.1 mm / 0.8° (`test_fase3_parity.py`) |
| 4 Misión y selección | e1be793 | 0.00 mm en pre-pick/pick; 5/5 pods en modo todos; paso a paso; abortar; gripper real (`test_fase4_mission.py`) |
| 5 Refinamiento EiH | 6a52e6a | fusión bit a bit (20/20 PPP); estimación EiH = PozoleV3; SINGLE/FUSED_2VIEW/PPP en la VM (`test_fase5_eih.py`) |
| 6 Web GUI | 9e18f10 | SIM y vivo en navegador headless; misión lanzada desde la GUI |
| **7 Calibración** | — | **en curso** (ver abajo) |

## Dónde estamos: Fase 7 (calibración)

Objetivo: `calibration_node` con el método de PozoleV3 (ChArUco 8×8 DICT_4X4_50 +
`cv2.calibrateHandEye`) para EtH (base ← cámara) y EiH (tool0 ← cam2); guardar en Config.xml por
serial (`config_manager/save_calibration`) y que `extrinsics_publisher` actualice el TF al momento;
pestaña de calibración en la GUI. Después, el paquete de Tecnalia como segundo backend.

Plan acordado con Michell: **validar el flujo** (captura de poses, secuencia, guardado) con la VM y
**comprobar el cálculo** contra PozoleV3 con los datasets de calibración (poses con imagen + TCP).

Hecho hasta ahora (sin código todavía): mapa completo del método de PozoleV3 en
`docs/pozolev3_calibracion.md` y datasets localizados (`PozoleV3/calibImages`, 21 poses por cámara).
Resultados preliminares, a verificar:
- cam0 y cam1: las matrices en uso se reproducen exactamente desde su dataset.
- **cam2: la matriz en uso no sale de su dataset** (123 mm de diferencia en la traslación). Hay que
  decidir con Michell cuál es la buena antes de recalibrar; la actual es la que dio buena fusión.

Siguiente paso: `calibration_node` (captura, detección, cálculo, guardado por serial), prueba de
paridad bit a bit con esos datasets, secuencia automática contra la VM y pestaña en la GUI.

## Pendiente

### Con el robot real (~2 semanas)
- **Prueba EiH de punta a punta** con cam2 en la muñeca: solo mirar (paso a paso, sin gripper) →
  repetibilidad de SINGLE sobre un mismo pod → comparar EtH / SINGLE / FUSED_2VIEW / PPP → cosecha.
  Poner `EIH/MaxMoveM` ≈ 0.05 para esa prueba.
- Revisar el offset de herramienta del AUBO real (la calibración de cam2 cuelga de `tool0`).
- Confirmar la pose **Release** (TCP a z = −0.196 m, bajo la base) y si el robot real también ignora
  objetivos articulares fuera de ±180° (la VM sí; arm_driver ya manda el equivalente).
- `robot:=real` (Robot/AuboReal en Config.xml).

### Software
- Fase 7 (calibración) y luego backend Tecnalia.
- `detector_node`: tras un `detect_once` sin imagen se queda suscrito a la RGB completa (CPU en vivo);
  convertirlo en instantánea como en `gui_bridge`.
- **Inestabilidad del EiH de PozoleV3**: en algunos pods, cambiar la nube 2 µm mueve el agarre 21–51 mm
  y el eje hasta 35° (vóxel del preproceso + remuestreo aleatorio de AdaPoinTr + mínimos locales del
  superquadric). No es de AtoleROS; revisar antes de fiarse de la corrección EiH (promediar capturas o
  semillas, o descartar por baja confianza).
- Backlog: preview "todos, uno a uno, simulado"; modelo de ripeness por ruta en Config.xml; MoveIt 2 +
  nvblox + cuRobo; lectura de fuerza del gripper; optimizaciones de Isaac Sim; `atole_lighting`
  (estrobos); `atole_localization` (u-blox RTK: el receptor está conectado, falta la antena); leer los
  seriales de cam0 y camC; prueba del swap real entre dos cámaras EtH.

## Hallazgos que conviene conocer
- **Dos ZED en el mismo namespace necesitan contenedores con nombre distinto**: con `zed_container`
  por defecto cada cámara se abría dos veces (Argus "Device in use", core dump de nvargus-daemon,
  cuelgue). AtoleROS usa `atole_cameras/launch/zed_camera.launch.py`.
- **El AUBO (VM) ignora sin error moveJoint con objetivos fuera de ±180°** aunque diga ±360°.
- **rclpy Jazzy**: `StaticTransformBroadcaster` no actualiza un frame ya enviado (→ `static_tf.py`);
  destruir suscripciones con el executor multihilo puede romper el wait set (→ executor de un hilo o
  suscripciones sin manejadores QoS por defecto).
- Imágenes de la ZED por DDS en **RELIABLE** (en best-effort se pierden casi todos los frames de 12 MB).
- Mask R-CNN idéntico a PozoleV3 exige backbone con FrozenBatchNorm2d y dividir /255 en CPU.
- El flag `invert_normal` de PozoleV3 no tenía efecto neto: la normal de aproximación siempre apunta arriba.
- La estimación AtoleROS reconstruye la nube desde profundidad + K; PozoleV3 leía la nube de la ZED.
  Difieren ~2 µm, y eso explica las pequeñas diferencias de paridad.

Referencias: `README.md`, `docs/pozolev3_eih.md` (refinamiento EiH de PozoleV3), `config/Config.xml`,
`tools/tests/`.
