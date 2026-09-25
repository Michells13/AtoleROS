"""Publica /robot_description y el árbol TF del brazo con robot_state_publisher."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory('atole_description')) / 'urdf' / 'aubo_is10.urdf'
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf.read_text()}], output='screen'),
    ])
