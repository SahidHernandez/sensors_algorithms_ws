import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _to_launch_value(value):
    if isinstance(value, bool):
        return "True" if value else "False"
    return str(value)


def _to_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _resolve_local_path(package_share, value):
    if not value:
        return value

    if os.path.isabs(value):
        return value

    direct_path = os.path.join(package_share, value)
    if os.path.exists(direct_path):
        return direct_path

    model_path = os.path.join(package_share, "models", value)
    if os.path.exists(model_path):
        return model_path

    return value


def _load_config(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    if not isinstance(config, dict):
        raise RuntimeError(f"Invalid YOLO config file: {config_file}")

    return config


def _launch_setup(context):
    package_share = get_package_share_directory("yolo_playground")
    bringup_share = get_package_share_directory("yolo_bringup")

    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_file)

    local_default_model = os.path.join(package_share, "models", "yolov8n.pt")
    default_model = local_default_model if os.path.exists(local_default_model) else "yolov8n.pt"

    launch_args = {
        "namespace": config.get("namespace", "yolo"),
        "model_type": config.get("model_type", "YOLO"),
        "model": _resolve_local_path(package_share, config.get("model", default_model)),
        "tracker": _resolve_local_path(package_share, config.get("tracker", "bytetrack.yaml")),
        "device": config.get("device", "cpu"),
        "fuse_model": config.get("fuse_model", False),
        "yolo_encoding": config.get("yolo_encoding", "bgr8"),
        "enable": config.get("enable", True),
        "threshold": config.get("threshold", 0.5),
        "iou": config.get("iou", 0.7),
        "imgsz_height": config.get("imgsz_height", 480),
        "imgsz_width": config.get("imgsz_width", 640),
        "half": config.get("half", False),
        "max_det": config.get("max_det", 300),
        "augment": config.get("augment", False),
        "agnostic_nms": config.get("agnostic_nms", False),
        "retina_masks": config.get("retina_masks", False),
        "input_image_topic": config.get("input_image_topic", "/camera/color/image_raw"),
        "image_reliability": config.get("image_reliability", 1),
        "input_depth_topic": config.get("input_depth_topic", "/camera/depth/image_raw"),
        "depth_image_reliability": config.get("depth_image_reliability", 1),
        "input_depth_info_topic": config.get("input_depth_info_topic", "/camera/depth/camera_info"),
        "depth_info_reliability": config.get("depth_info_reliability", 1),
        "target_frame": config.get("target_frame", "base_link"),
        "depth_image_units_divisor": config.get("depth_image_units_divisor", 1000),
        "use_tracking": config.get("use_tracking", True),
        "use_3d": config.get("use_3d", False),
        "use_debug": config.get("use_debug", True),
    }

    namespace_override = LaunchConfiguration("namespace").perform(context).strip()
    if namespace_override:
        launch_args["namespace"] = namespace_override

    model_override = LaunchConfiguration("model").perform(context).strip()
    if model_override:
        launch_args["model"] = _resolve_local_path(package_share, model_override)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "yolo.launch.py")
            ),
            launch_arguments={
                key: _to_launch_value(value) for key, value in launch_args.items()
            }.items(),
        )
    ]

    if _to_bool(LaunchConfiguration("start_detection_logger").perform(context)):
        actions.append(
            Node(
                package="yolo_playground",
                executable="detection_logger",
                name="detection_logger",
                namespace=launch_args["namespace"],
                parameters=[{"detections_topic": "detections", "report_period_sec": 2.0}],
            )
        )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory("yolo_playground")
    default_config_file = os.path.join(package_share, "config", "yolo_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to the YOLO playground YAML config file",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace override",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="Optional model override. Relative paths are resolved from this package.",
            ),
            DeclareLaunchArgument(
                "start_detection_logger",
                default_value="False",
                description="Start the example detection logger node",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
