from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('qr_detector')

    default_config_file = os.path.join(
        pkg_dir,
        'config',
        'qr_detector.yaml'
    )

    # -----------------------------
    # Argumentos
    # -----------------------------
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Archivo YAML de configuración del detector QR'
    )

    # -----------------------------
    # Nodo detector QR
    # -----------------------------
    qr_detector_node = Node(
        package='qr_detector',
        executable='qr_detector_node.py',
        name='qr_detector_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ]
    )

    return LaunchDescription([
        config_file_arg,
        qr_detector_node
    ])
