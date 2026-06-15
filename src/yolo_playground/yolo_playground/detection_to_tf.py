#!/usr/bin/env python3

import math
import re
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from yolo_msgs.msg import DetectionArray


def sanitize_tf_name(name: str) -> str:
    name = str(name).strip().lower()
    name = re.sub(r"[^a-zA-Z0-9_]+", "_", name)
    name = re.sub(r"_+", "_", name)
    name = name.strip("_")

    if not name:
        name = "object"

    return name


class DetectionsToTF(Node):
    def __init__(self):
        super().__init__("detections_to_tf")

        self.declare_parameter("detections_topic", "/yolo/detections_3d")
        self.declare_parameter("tf_prefix", "detected")
        self.declare_parameter("min_score", 0.5)
        self.declare_parameter("publish_only_best", False)
        self.declare_parameter("default_parent_frame", "camera_link")

        # Filtros de distancia.
        # En camera_link de RealSense, normalmente X positivo es hacia enfrente.
        self.declare_parameter("min_forward_distance", 0.25)
        self.declare_parameter("max_forward_distance", 4.0)

        # Suavizado del TF.
        # 0.0 = no se mueve, 1.0 = sin suavizado.
        self.declare_parameter("smoothing_alpha", 0.35)

        # Si True, mantiene el último TF válido cuando la detección 3D desaparece.
        self.declare_parameter("hold_last_valid_tf", True)
        self.declare_parameter("hold_timeout_sec", 0.7)

        self.detections_topic = (
            self.get_parameter("detections_topic").get_parameter_value().string_value
        )
        self.tf_prefix = (
            self.get_parameter("tf_prefix").get_parameter_value().string_value
        )
        self.min_score = (
            self.get_parameter("min_score").get_parameter_value().double_value
        )
        self.publish_only_best = (
            self.get_parameter("publish_only_best").get_parameter_value().bool_value
        )
        self.default_parent_frame = (
            self.get_parameter("default_parent_frame").get_parameter_value().string_value
        )
        self.min_forward_distance = (
            self.get_parameter("min_forward_distance").get_parameter_value().double_value
        )
        self.max_forward_distance = (
            self.get_parameter("max_forward_distance").get_parameter_value().double_value
        )
        self.smoothing_alpha = (
            self.get_parameter("smoothing_alpha").get_parameter_value().double_value
        )
        self.hold_last_valid_tf = (
            self.get_parameter("hold_last_valid_tf").get_parameter_value().bool_value
        )
        self.hold_timeout_sec = (
            self.get_parameter("hold_timeout_sec").get_parameter_value().double_value
        )

        self.smoothing_alpha = max(0.0, min(1.0, self.smoothing_alpha))

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_positions: Dict[str, Tuple[float, float, float]] = {}
        self.last_transforms: Dict[str, TransformStamped] = {}
        self.last_seen_time: Dict[str, rclpy.time.Time] = {}

        self.sub = self.create_subscription(
            DetectionArray,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.republish_last_valid_tfs)

        self.get_logger().info(f"Listening detections from: {self.detections_topic}")
        self.get_logger().info("Publishing stable detection TFs")

    def build_child_frame(self, det, index: int) -> str:
        class_name = sanitize_tf_name(det.class_name)

        if det.id != "":
            object_id = sanitize_tf_name(det.id)
        else:
            object_id = str(index)

        return f"{self.tf_prefix}_{class_name}_{object_id}"

    def is_valid_detection(self, det) -> bool:
        if det.score < self.min_score:
            return False

        bbox3d = det.bbox3d

        x = bbox3d.center.position.x
        y = bbox3d.center.position.y
        z = bbox3d.center.position.z

        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            return False

        # Para target_frame=camera_link, X suele ser distancia hacia enfrente.
        # Si el objeto está demasiado cerca, lo ignoramos para evitar brincos.
        if x < self.min_forward_distance:
            return False

        if x > self.max_forward_distance:
            return False

        return True

    def smooth_position(self, child_frame: str, x: float, y: float, z: float):
        if child_frame not in self.last_positions:
            self.last_positions[child_frame] = (x, y, z)
            return x, y, z

        last_x, last_y, last_z = self.last_positions[child_frame]
        a = self.smoothing_alpha

        sx = a * x + (1.0 - a) * last_x
        sy = a * y + (1.0 - a) * last_y
        sz = a * z + (1.0 - a) * last_z

        self.last_positions[child_frame] = (sx, sy, sz)

        return sx, sy, sz

    def detections_callback(self, msg: DetectionArray):
        valid_detections = []

        for det in msg.detections:
            if self.is_valid_detection(det):
                valid_detections.append(det)

        if not valid_detections:
            return

        if self.publish_only_best:
            valid_detections = [max(valid_detections, key=lambda d: d.score)]

        transforms = []

        for index, det in enumerate(valid_detections):
            bbox3d = det.bbox3d

            parent_frame = bbox3d.frame_id
            if parent_frame == "":
                parent_frame = self.default_parent_frame

            child_frame = self.build_child_frame(det, index)

            raw_x = bbox3d.center.position.x
            raw_y = bbox3d.center.position.y
            raw_z = bbox3d.center.position.z

            x, y, z = self.smooth_position(child_frame, raw_x, raw_y, raw_z)

            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = parent_frame
            transform.child_frame_id = child_frame

            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z

            transform.transform.rotation.x = bbox3d.center.orientation.x
            transform.transform.rotation.y = bbox3d.center.orientation.y
            transform.transform.rotation.z = bbox3d.center.orientation.z
            transform.transform.rotation.w = bbox3d.center.orientation.w

            if transform.transform.rotation.w == 0.0:
                transform.transform.rotation.w = 1.0

            transforms.append(transform)

            self.last_transforms[child_frame] = transform
            self.last_seen_time[child_frame] = self.get_clock().now()

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def republish_last_valid_tfs(self):
        if not self.hold_last_valid_tf:
            return

        now = self.get_clock().now()
        transforms = []

        for child_frame, transform in list(self.last_transforms.items()):
            last_seen = self.last_seen_time.get(child_frame)

            if last_seen is None:
                continue

            age = (now - last_seen).nanoseconds / 1e9

            if age <= self.hold_timeout_sec:
                transform.header.stamp = now.to_msg()
                transforms.append(transform)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionsToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()