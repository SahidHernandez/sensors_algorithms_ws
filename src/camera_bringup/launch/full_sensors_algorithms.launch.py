"""
Master launch file for the Sentinel sensor and algorithm suite.

This script orchestrates the complete startup sequence for all hardware sensors 
(RealSense, Fisheye, USB, Thermal) and software processing nodes (Landolt, Motion, 
QR, YOLO). It uses conditional launch arguments to selectively enable or disable 
specific modules and employs TimerActions to stagger the node initialization, 
ensuring system stability and preventing USB bandwidth or CPU overloads during boot.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generates the ROS 2 LaunchDescription for the complete Sentinel system.

    This function declares all configuration arguments (toggle flags and 
    YOLO-specific parameters) and constructs the ordered execution graph. 
    Nodes are launched in a heavily staggered sequence to allow hardware 
    interfaces to settle before algorithms attempt to subscribe to them:
    
    Sequence:
        1. Cameras (T=0s)
        2. Landolt Detector (T=10s)
        3. Motion Detector (T=12s)
        4. QR Detector (T=13s)
        5. YOLO 3D (T=15s)

    Returns:
        LaunchDescription: The populated launch description object containing 
            all declared arguments, conditional includes, and timers.
    """
    # --- Configuraciones de encendido/apagado ---
    use_cameras = LaunchConfiguration('use_cameras')
    use_landolt = LaunchConfiguration('use_landolt')
    use_motion = LaunchConfiguration('use_motion')
    use_qr = LaunchConfiguration('use_qr')
    use_yolo = LaunchConfiguration('use_yolo')

    # --- 1. Cámaras ---
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('camera_bringup'),
                'launch',
                'cameras.launch.py',
            ])
        ),
        condition=IfCondition(use_cameras),
    )

    # --- 2. Landolt ---
    landolt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('capra_landolt_ros'),
                'launch',
                'landolt_detection.launch.py',
            ])
        ),
        condition=IfCondition(use_landolt),
    )

    # --- 3. Motion ---
    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('motion_stability_detector'),
                'launch',
                'motion_detector.launch.py',
            ])
        ),
        condition=IfCondition(use_motion),
    )

    # --- 4. QR ---
    qr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('qr_detector'),
                'launch',
                'qr_detector.launch.py',
            ])
        ),
        condition=IfCondition(use_qr),
    )

    # --- 5. YOLO (Puro, sin Landmarks) ---
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolo.launch.py',
            ])
        ),
        condition=IfCondition(use_yolo),
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

    return LaunchDescription([
        # --- Declaración de variables de uso ---
        DeclareLaunchArgument('use_cameras', default_value='true', description='Lanzar cámaras.'),
        DeclareLaunchArgument('use_landolt', default_value='true', description='Lanzar detector Landolt.'),
        DeclareLaunchArgument('use_motion', default_value='true', description='Lanzar detector de movimiento.'),
        DeclareLaunchArgument('use_qr', default_value='true', description='Lanzar detector QR.'),
        DeclareLaunchArgument('use_yolo', default_value='true', description='Lanzar detector YOLO puro.'),

        # --- Declaración de argumentos para YOLO ---
        DeclareLaunchArgument('camera_frame_id', default_value='camera_link', description='Frame base usado por YOLO 3D.'),
        DeclareLaunchArgument(
            'yolo_model',
            default_value=PathJoinSubstitution([FindPackageShare('yolo_playground'), 'models', 'best.pt']),
            description='Ruta al modelo YOLO.'
        ),
        DeclareLaunchArgument('yolo_device', default_value='cpu', description='Dispositivo para YOLO, por ejemplo cpu o cuda:0.'),
        DeclareLaunchArgument('yolo_threshold', default_value='0.5', description='Threshold de deteccion para YOLO.'),
        DeclareLaunchArgument('yolo_iou', default_value='0.5', description='IoU para YOLO.'),
        DeclareLaunchArgument('yolo_max_det', default_value='50', description='Numero maximo de detecciones por frame.'),
        DeclareLaunchArgument('yolo_use_debug', default_value='True', description='Habilita el nodo debug de YOLO.'),
        DeclareLaunchArgument('image_reliability', default_value='2', description='QoS reliability de imagen RGB para YOLO.'),
        DeclareLaunchArgument('depth_image_reliability', default_value='2', description='QoS reliability de depth image para YOLO.'),
        DeclareLaunchArgument('depth_info_reliability', default_value='2', description='QoS reliability de depth camera_info para YOLO.'),

        # --- Secuencia de Ejecución ---
        
        # 1. Primero cámaras (Instantáneo)
        cameras_launch,

        # 2. Luego Landolt (10s)
        TimerAction(
            period=10.0,
            actions=[landolt_launch],
        ),

        # 3. Luego motion detector (12s)
        TimerAction(
            period=12.0,
            actions=[motion_launch],
        ),

        # 4. Luego QR detector (13s)
        TimerAction(
            period=13.0,
            actions=[qr_launch],
        ),

        # 5. Finalmente YOLO (15s)
        TimerAction(
            period=15.0,
            actions=[yolo_launch],
        ),
    ])