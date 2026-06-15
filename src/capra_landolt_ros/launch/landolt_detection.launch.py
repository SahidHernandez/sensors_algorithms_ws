from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("capra_landolt_ros")

    params_file = os.path.join(
        pkg_share,
        "config",
        "landolt_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="capra_landolt_ros",
            executable="landolt_node",
            name="landolt_node",
            output="screen",
            parameters=[params_file],
        )
    ])
