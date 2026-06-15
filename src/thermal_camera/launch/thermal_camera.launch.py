from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('thermal_camera')
    params_file = os.path.join(pkg_share, 'config', 'thermal_camera.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0',
            description='Dispositivo V4L2 persistente de la camara termica.',
        ),
        Node(
            package='thermal_camera',
            executable='thermal_camera_node',
            name='thermal_camera_node',
            output='screen',
            parameters=[params_file, {'video_device': LaunchConfiguration('video_device')}],
        ),
    ])
