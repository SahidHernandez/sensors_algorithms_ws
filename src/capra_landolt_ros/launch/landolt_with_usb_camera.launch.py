from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("capra_landolt_ros"),
            "rviz",
            "landolt_ros.rviz"
        ]),
        description="Path to RViz config file"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2"
    )

    landolt_node = Node(
        package="capra_landolt_ros",
        executable="landolt_node",
        name="landolt_node",
        output="screen",
        parameters=[
            {
                "camera_topic": "/camera/color/image_raw",
                "threshold_value": 140,
                "min_edge": 12,
                "min_ratio_circle": 0.8,
                "min_depth": 10,
                "publish_debug_image": True,
                "crop_scale": 1.5,
                "choose_largest_detection": True,
                "min_sharpness": 350.0,
                "show_rejected_blur_warning": True,
                "robocup_task_mode": "generic",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config")
        ],
    )

    return LaunchDescription([
        rviz_config_arg,
        use_rviz_arg,
        landolt_node,
        rviz_node,
    ])
