"""Arranque de AtoleROS.

ros2 launch atole_bringup atole.launch.py robot:=mock sim:=false gui:=true
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from atole_bringup.launch_utils import DEFAULT_VENV, venv_node

DEFAULT_CONFIG = str(Path.home() / 'Documents/pre-Alpha/AtoleROS/config/Config.xml')

# Nodos que necesitan el venv (torch, numpy 2, pyaubo_sdk, minimalmodbus...).
VENV_NODES = [
    ('atole_arm', 'arm_driver'),
    ('atole_gripper', 'grip_twist_controller'),
    ('atole_perception', 'detector_node'),
    ('atole_perception', 'pod_pose_node'),
    ('atole_perception', 'cloud_fusion_node'),
    ('atole_task_planning', 'pod_selector'),
    ('atole_mission', 'mission_manager'),
    ('atole_calibration', 'calibration_node'),
    ('atole_gui', 'gui_bridge'),
]
# Nodos que corren con el Python del sistema.
SYSTEM_NODES = [
    ('atole_system', 'system_monitor'),
    ('atole_cameras', 'camera_manager'),
    ('atole_cameras', 'extrinsics_publisher'),
]


def _nodes(context):
    robot = LaunchConfiguration('robot').perform(context)
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    venv = os.path.expanduser(LaunchConfiguration('venv').perform(context))
    config = os.path.expanduser(LaunchConfiguration('config').perform(context))

    actions = [Node(package='atole_config', executable='config_manager', output='screen',
                    parameters=[{'config_path': config}])]
    for p, e in SYSTEM_NODES:
        params = [{'sim': sim}] if p == 'atole_cameras' else []
        # camera_manager cierra las ZED al salir (~15 s cada una): margen antes de SIGTERM/SIGKILL.
        timeouts = {'sigterm_timeout': '40', 'sigkill_timeout': '10'} if e == 'camera_manager' else {}
        actions.append(Node(package=p, executable=e, output='screen', parameters=params, **timeouts))
    for package, executable in VENV_NODES:
        params = {'backend': robot} if executable == 'arm_driver' else None
        actions.append(venv_node(package, executable, venv, params))
    if sim:
        actions.append(venv_node('atole_sim', 'dataset_player', venv))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('atole_description')) / 'launch' / 'description.launch.py'))))
    if gui:
        rosbridge = Path(get_package_share_directory('rosbridge_server')) / 'launch' / 'rosbridge_websocket_launch.xml'
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(str(rosbridge))))
        actions.append(Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen',
                            parameters=[{'port': 8765}]))
    return actions


def generate_launch_description():
    dds = Path(get_package_share_directory('atole_bringup')) / 'config' / 'fastdds_nonblocking.xml'
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='mock', description='mock | vm | real'),
        DeclareLaunchArgument('sim', default_value='false', description='true = modo SIM (dataset_player)'),
        DeclareLaunchArgument('gui', default_value='true', description='rosbridge (:9090) + foxglove_bridge (:8765)'),
        DeclareLaunchArgument('venv', default_value=DEFAULT_VENV),
        DeclareLaunchArgument('config', default_value=DEFAULT_CONFIG),
        # DDS local y con envío no bloqueante (evita el bloqueo por el gadget micro-USB).
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(dds)),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        OpaqueFunction(function=_nodes),
    ])
