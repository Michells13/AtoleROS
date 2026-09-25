# AtoleROS

Sistema de cosecha de cacao en ROS 2 Jazzy (ZED Box Duo, AUBO iS10, gripper RGI-100).
Diseñado desde cero; PozoleV3 se usa solo como guía. Arquitectura y grafo interactivo:
https://claude.ai/artifact/LWy6wXZ8wXN6hzgK11eD3j

## Estado
**Fase 0 (esqueleto):** los 15 paquetes compilan y todos los nodos arrancan con su interfaz
definitiva (topics, servicios y acciones). Salvo `config_manager`, los servicios y acciones
responden "no implementado todavía (Fase N)".

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
