"""Una ZED con su propio contenedor: /atole/zed/zed_container_<camera_name>.

El launch oficial (zed_wrapper/zed_camera.launch.py) crea siempre un contenedor llamado
`zed_container` si no se le da otro. Con dos cámaras en el mismo namespace los dos contenedores se
llaman igual, los dos atienden la petición load_node y cada cámara se abre dos veces (Argus
"Device in use", segfaults, caída de nvargus-daemon y cuelgue de la ZED Box). Aquí se crea un
contenedor con nombre propio y se incluye el launch oficial apuntando a él. El contenedor es hijo
de este launch, así que al cerrarlo se espera a que la ZED se libere.

Argumentos: los del launch oficial (camera_model, camera_name, namespace, serial_number, ...).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer

PASSTHROUGH = ['camera_model', 'serial_number', 'publish_urdf', 'publish_tf', 'publish_map_tf', 'param_overrides']


def setup(context):
    cam = LaunchConfiguration('camera_name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    container = f'zed_container_{cam}'
    args = {k: LaunchConfiguration(k).perform(context) for k in PASSTHROUGH}
    args = {k: v for k, v in args.items() if v != ''}
    args.update(camera_name=cam, namespace=namespace, container_name=container)
    official = Path(get_package_share_directory('zed_wrapper')) / 'launch' / 'zed_camera.launch.py'
    return [
        ComposableNodeContainer(name=container, namespace=namespace, package='rclcpp_components',
                                executable='component_container_isolated', composable_node_descriptions=[],
                                arguments=['--use_multi_threaded_executor', '--ros-args', '--log-level', 'info'],
                                output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(official)), launch_arguments=args.items()),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument('camera_name'), DeclareLaunchArgument('namespace', default_value='atole/zed')]
        + [DeclareLaunchArgument(k, default_value='') for k in PASSTHROUGH]
        + [OpaqueFunction(function=setup)])
