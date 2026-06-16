"""
Launch file for the Sentinel camera suite.

This script manages the concurrent initialization of multiple cameras 
(RealSense, Fisheye, USB, and Thermal) for the Sentinel robot. It parses 
configurations from a YAML file and uses TimerActions to stagger the 
startup sequence, preventing USB bus overload.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def load_yaml(package_name, relative_path):
    """
    Loads and parses a YAML configuration file from a ROS 2 package.

    Args:
        package_name (str): The name of the ROS 2 package containing the file.
        relative_path (list of str): The path components inside the package's 
            share directory (e.g., ['config', 'params.yaml']).

    Returns:
        dict: The parsed YAML data as a Python dictionary. Defaults to an 
            empty dictionary if the file is empty.

    Raises:
        FileNotFoundError: If the specified YAML file does not exist.
    """
    config_file = os.path.join(
        get_package_share_directory(package_name),
        *relative_path,
    )

    if not os.path.exists(config_file):
        raise FileNotFoundError(f'No se encontró el archivo de configuración: {config_file}')

    with open(config_file, 'r', encoding='utf-8') as file_handle:
        return yaml.safe_load(file_handle) or {}


def bool_to_launch(value):
    """
    Converts a boolean value to a ROS 2 launch-compatible string.

    Args:
        value (bool): The boolean value to convert.

    Returns:
        str: 'true' or 'false' in lowercase.
    """
    return str(bool(value)).lower()


def value_to_launch(value):
    """
    Converts a generic Python value to a string for ROS 2 arguments.

    Args:
        value (Any): The value to convert.

    Returns:
        str: The string representation of the value.
    """
    return str(value)


def serial_to_launch(value):
    """
    Formats a camera serial number for the realsense2_camera node.

    The RealSense ROS 2 wrapper requires purely numeric serial numbers 
    to be prefixed with an underscore ('_') to prevent them from being 
    parsed as floats/integers by the launch system.

    Args:
        value (Union[str, int, None]): The raw serial number.

    Returns:
        str: The formatted serial number string, or "''" if empty.
    """
    if value is None:
        return "''"

    serial = str(value).strip()
    if not serial:
        return "''"

    # realsense2_camera expects numeric serials as strings prefixed with '_'
    if serial[0].isdigit() and not serial.startswith('_'):
        return f'_{serial}'

    return serial


def generate_launch_description():
    """
    Generates the ROS 2 LaunchDescription for the camera bringup.

    Reads parameters from `camera_params.yaml` and constructs the launch 
    sequence for the RealSense D455, Fisheye camera, UGREEN USB camera, 
    and Thermal camera. Timers are used to stagger the node executions 
    by 4-second intervals.

    Returns:
        LaunchDescription: The populated launch description object.
    """
    # =========================
    # Load camera_bringup config
    # =========================
    camera_params = load_yaml(
        'camera_bringup',
        ['config', 'camera_params.yaml'],
    )

    cameras_config = camera_params.get('cameras', {})

    realsense_config = cameras_config.get('realsense', {})
    fisheye_config = cameras_config.get('fisheye', {})
    usb_config = cameras_config.get('usb_camera', {})
    thermal_config = cameras_config.get('thermal_camera', {})

    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py',
    )
    fisheye_launch_file = os.path.join(
        get_package_share_directory('fisheye_camera'),
        'launch',
        'fisheye_camera.launch.py',
    )
    thermal_launch_file = os.path.join(
        get_package_share_directory('thermal_camera'),
        'launch',
        'thermal_camera.launch.py',
    )

    # =========================
    # RealSense D455
    # =========================
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'camera_name': realsense_config.get('camera_name', 'camera'),
            'camera_namespace': realsense_config.get('camera_namespace', ''),
            'serial_no': serial_to_launch(realsense_config.get('serial_no', "''")),

            # RGB
            'enable_color': bool_to_launch(
                realsense_config.get('enable_color', True)
            ),
            'rgb_camera.color_profile': realsense_config.get(
                'color_profile',
                '640x480x15',
            ),

            # Depth
            'enable_depth': bool_to_launch(
                realsense_config.get('enable_depth', False)
            ),
            'depth_module.depth_profile': realsense_config.get(
                'depth_profile',
                '640x480x15',
            ),

            # Depth alineado a color
            'align_depth.enable': bool_to_launch(
                realsense_config.get('align_depth_enable', False)
            ),

            # PointCloud
            'pointcloud.enable': bool_to_launch(
                realsense_config.get('pointcloud_enable', False)
            ),

            # Opcionales útiles
            'enable_sync': bool_to_launch(
                realsense_config.get('enable_sync', False)
            ),
            'publish_tf': bool_to_launch(
                realsense_config.get('publish_tf', True)
            ),
            'tf_publish_rate': value_to_launch(
                realsense_config.get('tf_publish_rate', 0.0)
            ),
            'initial_reset': bool_to_launch(
                realsense_config.get('initial_reset', False)
            ),
            'reconnect_timeout': value_to_launch(
                realsense_config.get('reconnect_timeout', 6.0)
            ),
            'wait_for_device_timeout': value_to_launch(
                realsense_config.get('wait_for_device_timeout', -1.0)
            ),
        }.items(),
    )

    # =========================
    # Fisheye camera
    # =========================
    # La fisheye queda controlada por su propio paquete/YAML.
    # Si fisheye_camera.launch.py ya carga su config interna,
    # no necesitamos pasarle parámetros aquí.
    fisheye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fisheye_launch_file),
        launch_arguments={
            'video_device': fisheye_config.get(
                'video_device',
                '/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0',
            ),
            'framerate': value_to_launch(fisheye_config.get('framerate', 15.0)),
            'pixel_format': fisheye_config.get('pixel_format', 'YUYV'),
            'output_encoding': fisheye_config.get('output_encoding', 'rgb8'),
        }.items(),
    )

    # =========================
    # Thermal camera
    # =========================
    thermal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thermal_launch_file),
        launch_arguments={
            'video_device': thermal_config.get(
                'video_device',
                '/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0',
            ),
        }.items(),
    )

    # =========================
    # UGREEN USB camera
    # =========================
    usb_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        output='screen',
        parameters=[
            {
                'video_device': usb_config.get(
                    'video_device',
                    '/dev/v4l/by-id/usb-UGREEN_UGREEN_camera_2K_AN202402200003-video-index0',
                ),
                'image_size': usb_config.get('image_size', [432, 240]),
                'framerate': usb_config.get('framerate', 15.0),
                'pixel_format': usb_config.get('pixel_format', 'MJPG'),
                'output_encoding': usb_config.get('output_encoding', 'rgb8'),
                'camera_frame_id': usb_config.get('camera_frame_id', 'usb_camera_link'),
            }
        ],
        ros_arguments=[
            '--ros-args',
            '-p', 'qos_overrides./usb_camera/image_raw.publisher.reliability:=best_effort',
            '-p', 'qos_overrides./usb_camera/image_raw.publisher.depth:=1',
            '-p', 'qos_overrides./usb_camera/camera_info.publisher.reliability:=best_effort',
            '-p', 'qos_overrides./usb_camera/camera_info.publisher.depth:=1',
        ],
        remappings=[
            ('image_raw', usb_config.get('image_topic', '/usb_camera/image_raw')),
            ('camera_info', usb_config.get('camera_info_topic', '/usb_camera/camera_info')),
        ],
    )

    # =========================
    # Launch sequence
    # =========================
    return LaunchDescription([
        realsense_launch,

        TimerAction(
            period=4.0,
            actions=[fisheye_launch],
        ),

        TimerAction(
            period=8.0,
            actions=[usb_camera_node],
        ),

        TimerAction(
            period=12.0,
            actions=[thermal_launch],
        ),
    ])