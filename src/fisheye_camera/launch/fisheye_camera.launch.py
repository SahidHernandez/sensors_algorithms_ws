"""
Launch file for the Fisheye Camera.

This script configures and launches the `v4l2_camera_node` specifically 
for the robot's fisheye camera using the Video4Linux2 driver. It dynamically 
loads camera calibration data, configures frame rates and pixel formats, 
and applies 'best_effort' Quality of Service (QoS) overrides necessary for 
smooth, low-latency video streaming in ROS 2 Humble.

Note:
    The `image_proc/rectify_node` is currently disabled because a placeholder 
    calibration is in use (distortion [0,0,0,0,0]). It should be uncommented 
    and re-enabled once a proper intrinsic calibration is performed using 
    the cameracalibrator tool.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generates the ROS 2 LaunchDescription for the fisheye camera.

    Declares launch arguments for the device path, framerate, pixel format, 
    and encoding, allowing them to be overridden from the command line or 
    master launch files. It constructs the v4l2 node with specific QoS 
    overrides passed directly as ROS arguments, bypassing a known limitation 
    with parameter overrides in ROS 2 Humble.

    Returns:
        LaunchDescription: The populated launch description containing 
            the configured arguments and the camera node.
    """
    pkg_share = FindPackageShare('fisheye_camera')

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0',
        description='Dispositivo V4L2 de la cámara fisheye'
    )
    framerate_arg = DeclareLaunchArgument(
        'framerate', default_value='15.0',
        description='Framerate objetivo'
    )
    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format', default_value='YUYV',
        description='Formato de pixel V4L2 para la camara fisheye'
    )
    output_encoding_arg = DeclareLaunchArgument(
        'output_encoding', default_value='rgb8',
        description='Encoding de salida ROS para la camara fisheye'
    )

    calib_file = PathJoinSubstitution([pkg_share, 'camera_info', 'fisheye_camera.yaml'])
    calib_url = ['file://', calib_file]

    # QoS: best_effort + depth 1 via ros arguments (qos_overrides como parámetros
    # de nodo NO funcionan en Humble — hay que pasarlos como ros_arguments)
    v4l2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        namespace='fisheye',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'camera_params.yaml']),
            {
                'video_device':    LaunchConfiguration('video_device'),
                'framerate':       LaunchConfiguration('framerate'),
                'pixel_format':    LaunchConfiguration('pixel_format'),
                'output_encoding': LaunchConfiguration('output_encoding'),
                'camera_info_url': calib_url,
                'camera_frame_id': 'fisheye_camera_link',
            },
        ],
        ros_arguments=[
            '--ros-args',
            '-p', 'qos_overrides./fisheye/image_raw.publisher.reliability:=best_effort',
            '-p', 'qos_overrides./fisheye/image_raw.publisher.depth:=1',
            '-p', 'qos_overrides./fisheye/camera_info.publisher.reliability:=best_effort',
            '-p', 'qos_overrides./fisheye/camera_info.publisher.depth:=1',
        ],
        remappings=[
            ('image_raw',   '/fisheye/image_raw'),
            ('camera_info', '/fisheye/camera_info'),
        ]
    )

    # NOTA: rectify_node deshabilitado mientras se usa calibración placeholder
    # Con distorsión [0,0,0,0,0] el rectify no hace ninguna corrección útil
    # y el TimeSynchronizer descarta frames por mismatch de QoS en Humble.
    # Re-habilitar después de calibración real con cameracalibrator.
    #
    # rectify_node = Node(
    #     package='image_proc',
    #     executable='rectify_node',
    #     ...
    # )

    return LaunchDescription([
        video_device_arg,
        framerate_arg,
        pixel_format_arg,
        output_encoding_arg,
        v4l2_node,
        # rectify_node,  # re-habilitar con calibración real
    ])