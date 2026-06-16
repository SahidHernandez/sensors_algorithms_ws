"""
Launch file for SLAM and Semantic Landmark Mapping.

This script integrates the camera bringup, RTAB-Map for Visual SLAM, 
YOLO for 3D object detection, and a custom landmark saver node. 
It creates a pipeline where RTAB-Map generates the spatial map and 
odometry, while YOLO detects objects in 3D space. The landmark saver 
then filters and anchors these detections into the global map frame 
as persistent semantic landmarks.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generates the ROS 2 LaunchDescription for the semantic mapping stack.

    This function sets up the following components:
    1. Base Cameras: Starts the camera suite (RealSense, etc.).
    2. RTAB-Map: Configured for RGB-D visual SLAM, syncing depth and color,
       and automatically resetting its database on startup.
    3. YOLO 3D: Runs object detection and projects bounding boxes into 3D space 
       using the aligned depth map.
    4. Landmark Saver: A custom node that filters 3D detections based on 
       confidence scores, distances, and confirmation timeouts to save stable 
       landmarks into a JSON file.
    5. Reset Process: An optional conditional process to clear the old landmarks 
       file before starting.

    Returns:
        LaunchDescription: The populated launch description object containing 
            all declarations, includes, and nodes for the SLAM pipeline.
    """
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('camera_bringup'),
                'launch',
                'cameras.launch.py',
            ])
        )
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py',
            ])
        ),
        launch_arguments={
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'frame_id': LaunchConfiguration('camera_frame_id'),
            'approx_sync': 'true',
            'visual_odometry': 'true',
            'rtabmap_args': '--delete_db_on_start',
        }.items(),
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolo.launch.py',
            ])
        ),
        launch_arguments={
            'input_image_topic': '/camera/color/image_raw',
            'input_depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'input_depth_info_topic': '/camera/aligned_depth_to_color/camera_info',
            'model': LaunchConfiguration('yolo_model'),
            'device': LaunchConfiguration('yolo_device'),
            'threshold': LaunchConfiguration('yolo_threshold'),
            'iou': LaunchConfiguration('yolo_iou'),
            'max_det': LaunchConfiguration('yolo_max_det'),
            'use_debug': LaunchConfiguration('yolo_use_debug'),
            'use_3d': 'True',
            'target_frame': LaunchConfiguration('camera_frame_id'),
            'image_reliability': LaunchConfiguration('image_reliability'),
            'depth_image_reliability': LaunchConfiguration('depth_image_reliability'),
            'depth_info_reliability': LaunchConfiguration('depth_info_reliability'),
        }.items(),
    )

    reset_landmarks = ExecuteProcess(
        cmd=['rm', '-f', LaunchConfiguration('landmarks_file')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('reset_landmarks')),
    )

    landmark_saver_node = Node(
        package='yolo_playground',
        executable='landmark_saver',
        name='landmark_saver',
        output='screen',
        parameters=[
            {
                'detections_topic': '/yolo/detections_3d',
                'map_frame': LaunchConfiguration('map_frame'),
                'landmarks_file': LaunchConfiguration('landmarks_file'),
                'min_score': LaunchConfiguration('landmark_min_score'),
                'min_confirmations': LaunchConfiguration('landmark_min_confirmations'),
                'confirmation_timeout_sec': LaunchConfiguration('landmark_confirmation_timeout_sec'),
                'candidate_merge_radius': LaunchConfiguration('candidate_merge_radius'),
                'landmark_merge_radius': LaunchConfiguration('landmark_merge_radius'),
                'min_distance': LaunchConfiguration('landmark_min_distance'),
                'max_distance': LaunchConfiguration('landmark_max_distance'),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='Frame base usado por RTAB-Map y YOLO 3D.',
        ),
        DeclareLaunchArgument(
            'yolo_model',
            default_value=PathJoinSubstitution([
                FindPackageShare('yolo_playground'),
                'models',
                'best.pt',
            ]),
            description='Ruta al modelo YOLO.',
        ),
        DeclareLaunchArgument(
            'yolo_device',
            default_value='cpu',
            description='Dispositivo para YOLO, por ejemplo cpu o cuda:0.',
        ),
        DeclareLaunchArgument(
            'yolo_threshold',
            default_value='0.5',
            description='Threshold de deteccion para YOLO.',
        ),
        DeclareLaunchArgument(
            'yolo_iou',
            default_value='0.5',
            description='IoU para YOLO.',
        ),
        DeclareLaunchArgument(
            'yolo_max_det',
            default_value='50',
            description='Numero maximo de detecciones por frame.',
        ),
        DeclareLaunchArgument(
            'yolo_use_debug',
            default_value='True',
            description='Habilita el nodo debug de YOLO.',
        ),
        DeclareLaunchArgument(
            'image_reliability',
            default_value='2',
            description='QoS reliability de imagen RGB para YOLO.',
        ),
        DeclareLaunchArgument(
            'depth_image_reliability',
            default_value='2',
            description='QoS reliability de depth image para YOLO.',
        ),
        DeclareLaunchArgument(
            'depth_info_reliability',
            default_value='2',
            description='QoS reliability de depth camera_info para YOLO.',
        ),
        DeclareLaunchArgument(
            'reset_landmarks',
            default_value='false',
            description='Borra el archivo de landmarks antes de iniciar.',
        ),
        DeclareLaunchArgument(
            'landmarks_file',
            default_value='/home/thesamayan/sensors_algorithms_ws/yolo_landmarks_map.json',
            description='Archivo JSON donde se guardan los landmarks.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Frame de referencia para guardar landmarks.',
        ),
        DeclareLaunchArgument(
            'landmark_min_score',
            default_value='0.82',
            description='Score minimo para aceptar landmarks.',
        ),
        DeclareLaunchArgument(
            'landmark_min_confirmations',
            default_value='18',
            description='Confirmaciones minimas para fijar un landmark.',
        ),
        DeclareLaunchArgument(
            'landmark_confirmation_timeout_sec',
            default_value='5.0',
            description='Timeout de confirmacion de candidatos.',
        ),
        DeclareLaunchArgument(
            'candidate_merge_radius',
            default_value='0.55',
            description='Radio de merge para candidatos.',
        ),
        DeclareLaunchArgument(
            'landmark_merge_radius',
            default_value='1.00',
            description='Radio de merge para landmarks consolidados.',
        ),
        DeclareLaunchArgument(
            'landmark_min_distance',
            default_value='0.40',
            description='Distancia minima valida para landmarks.',
        ),
        DeclareLaunchArgument(
            'landmark_max_distance',
            default_value='4.0',
            description='Distancia maxima valida para landmarks.',
        ),
        cameras_launch,
        rtabmap_launch,
        yolo_launch,
        reset_landmarks,
        landmark_saver_node,
    ])