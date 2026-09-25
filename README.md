# AtoleROS

Sistema de cosecha de cacao en ROS 2 Jazzy (ZED Box Duo, AUBO iS10, gripper RGI-100).
Diseñado desde cero; PozoleV3 se usa solo como guía. Arquitectura y grafo interactivo:
https://claude.ai/artifact/LWy6wXZ8wXN6hzgK11eD3j

## Estado
- **Fase 0 (esqueleto):** los 15 paquetes compilan y todos los nodos arrancan con su interfaz definitiva.
- **Fase 1 (robot y gripper):**
  - `arm_driver` funcional contra la VM del AUBO: `/joint_states` a 20 Hz, lease, movimientos
    cancelables, IK de frente y moveL con J6 bloqueado. Pruebas en `tools/tests/test_fase1_arm.py`.
  - `grip_twist_controller` funcional con el RGI-100 real: 1000 = abierto, 0 = cerrado;
    rotación, feedback real de posición y ángulo.
  - `system_monitor` con checks reales (READY).
- **Fase 2 (cámaras, TF y SIM):**
  - `camera_manager` lanza la EtH seleccionada + cam2 con zed-ros2-wrapper
    (`/atole/zed/<cam>/...`), cambia de EtH (`/atole/cameras/select_eth`) y cierra sin huérfanos.
  - `extrinsics_publisher` publica `<cam>_calibrated_optical` (matriz exacta de Config.xml,
    solo si coincide el serial). Es el marco de referencia para la percepción.
  - `dataset_player` (`sim:=true`): `/atole/sim/list|load|stop` publica un dataset EtH
    (cam0 + cam1, el mismo de PozoleV3) con los mismos topics, marcos y stamps que la ZED.
- **Fase 3 (percepción EtH/EiH):**
  - `detector_node`: Mask R-CNN (torchvision) con el modelo de Config.xml; cam2 usa el modelo EiH.
    `detect_once` (con imagen opcional), detección continua y warm-up.
  - `pod_pose_node`: acción `estimate_pods` → frame sincronizado → máscaras → nube parcial →
    filtro de área de trabajo → AdaPoinTr → superquadric → PCA → pods en `base_link`. Publica
    pods, nubes (parcial y completada) y marcadores.
  - Warm-up al arrancar (Mask R-CNN y AdaPoinTr, EtH y EiH): el sistema no pasa a READY hasta que termina.
  - Paridad con PozoleV3 (`tools/tests/test_fase3_parity.py`, 43 detecciones): detecciones, scores
    y máscaras idénticos; pose ≤ 1.1 mm y 0.8°, diferencia que viene solo de reconstruir la nube desde
    profundidad + K (con la misma nube: 0.001 mm y 0.05°, ruido de GPU).
- **Fase 4 (misión y selección):**
  - `pod_selector`: estrategias proximity | left_to_right | right_to_left | confidence y filtro de
    alcanzabilidad (IK de frente del pre-pick y del pick).
  - `mission_manager`: acción `/atole/mission/harvest` (todos | siguiente, paso a paso con
    `/atole/mission/confirm_step`, `/atole/mission/abort`). Home2 → detectar → seleccionar → pre-pick →
    pick → cerrar → rotar → retirar → Release → Home2, con las velocidades y la regla de aproximación
    de PozoleV3 (`atole_common/grasp.py`). Pruebas en `tools/tests/test_fase4_mission.py`.
  - `arm_driver`: los objetivos articulares se llevan a su equivalente dentro de ±180° (el AUBO acepta
    y no ejecuta, sin error, objetivos fuera de ese rango).
- **Fase 5 (refinamiento EiH):**
  - Estrategias SINGLE, FUSED_2VIEW y PPP como los botones de PozoleV3 (`docs/pozolev3_eih.md`):
    captura de cam2, asociación con el pod EtH, completion con el checkpoint EiH, fusión voxel-merge
    (`cloud_fusion_node`) y corrección del pre-pick y el pick (`atole_common/eih.py`).
  - SIM EiH: `/atole/sim/list_eih|load_eih` reproduce 240 vistas reales de cam2 con la pose del TCP grabada.
  - Paridad (`tools/tests/test_fase5_eih.py`): voxel-merge idéntico bit a bit (20/20 pares PPP) y estimación
    EiH idéntica a PozoleV3 con la misma nube de entrada.
  - Los servicios y acciones de las fases siguientes siguen respondiendo "no implementado todavía (Fase N)".

## Compilar
```bash
cd ~/Documents/pre-Alpha/AtoleROS
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Arrancar
```bash
source install/setup.bash
ros2 launch atole_bringup atole.launch.py robot:=mock sim:=false gui:=true
```
Argumentos:
- `robot:=mock|vm|real`
- `sim:=true` (modo SIM con `dataset_player`)
- `gui:=true` (rosbridge :9090 + foxglove_bridge :8765 para Lichtblick)
- `config:=<ruta a Config.xml>`
- `venv:=~/venvs/main312`

Para usar la CLI de ROS contra el sistema, exporta el mismo entorno DDS que el launch:
```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/install/atole_bringup/share/atole_bringup/config/fastdds_nonblocking.xml
```

## Configuración
`config/Config.xml` es la única fuente de configuración (cámaras, calibraciones por serial,
robot, gripper, poses Home/Home2/Release, cosecha, EiH, percepción, selección, calibración, SIM).
Solo `config_manager` lo escribe (servicios `/atole/config/get|set|save_pose|save_calibration`),
de forma atómica y con copia `Config.xml.bak`.

## Paquetes
| Paquete | Nodos | Python |
|---|---|---|
| atole_interfaces | — (13 msg, 20 srv, 7 action) | — |
| atole_common | — (stubs, QoS, ConfigView) | — |
| atole_config | config_manager | sistema |
| atole_system | system_monitor | sistema |
| atole_cameras | camera_manager, extrinsics_publisher | sistema |
| atole_description | robot_state_publisher (URDF AUBO iS10) | — |
| atole_sim | dataset_player | venv |
| atole_perception | detector_node, pod_pose_node, cloud_fusion_node | venv |
| atole_task_planning | pod_selector | venv |
| atole_arm | arm_driver | venv |
| atole_gripper | grip_twist_controller | venv |
| atole_mission | mission_manager | venv |
| atole_calibration | calibration_node | venv |
| atole_gui | gui_bridge | venv |
| atole_bringup | launch, perfil Fast DDS | — |
