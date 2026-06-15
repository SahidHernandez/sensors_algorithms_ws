#!/usr/bin/env python3

import json
import math
import os
import re
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener

from yolo_msgs.msg import DetectionArray


def sanitize_name(name: str) -> str:
    name = str(name).strip().lower()
    name = re.sub(r"[^a-zA-Z0-9_]+", "_", name)
    name = re.sub(r"_+", "_", name)
    name = name.strip("_")
    return name if name else "object"


def distance_3d(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def quat_rotate_vector(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    vx: float,
    vy: float,
    vz: float,
) -> Tuple[float, float, float]:
    """
    Rota un vector usando quaternion.
    q debe venir como x, y, z, w.
    """
    # u = vector part, s = scalar part
    ux, uy, uz = qx, qy, qz
    s = qw

    dot_uv = ux * vx + uy * vy + uz * vz
    dot_uu = ux * ux + uy * uy + uz * uz

    cross_x = uy * vz - uz * vy
    cross_y = uz * vx - ux * vz
    cross_z = ux * vy - uy * vx

    rx = 2.0 * dot_uv * ux + (s * s - dot_uu) * vx + 2.0 * s * cross_x
    ry = 2.0 * dot_uv * uy + (s * s - dot_uu) * vy + 2.0 * s * cross_y
    rz = 2.0 * dot_uv * uz + (s * s - dot_uu) * vz + 2.0 * s * cross_z

    return rx, ry, rz


class LandmarkSaver(Node):
    def __init__(self):
        super().__init__("landmark_saver")

        self.declare_parameter("detections_topic", "/yolo/detections_3d")
        self.declare_parameter("map_frame", "camera_link")
        self.declare_parameter("landmarks_file", os.path.expanduser("~/.ros/yolo_landmarks.json"))

        self.declare_parameter("min_score", 0.75)
        self.declare_parameter("min_confirmations", 8)
        self.declare_parameter("confirmation_timeout_sec", 3.0)

        self.declare_parameter("candidate_merge_radius", 0.20)
        self.declare_parameter("landmark_merge_radius", 0.30)

        self.declare_parameter("max_distance", 4.0)
        self.declare_parameter("min_distance", 0.30)

        self.declare_parameter("class_whitelist_csv", "")
        self.declare_parameter("publish_markers", True)

        self.detections_topic = self.get_parameter(
            "detections_topic"
        ).get_parameter_value().string_value

        self.map_frame = self.get_parameter(
            "map_frame"
        ).get_parameter_value().string_value

        self.landmarks_file = self.get_parameter(
            "landmarks_file"
        ).get_parameter_value().string_value

        self.min_score = self.get_parameter(
            "min_score"
        ).get_parameter_value().double_value

        self.min_confirmations = self.get_parameter(
            "min_confirmations"
        ).get_parameter_value().integer_value

        self.confirmation_timeout_sec = self.get_parameter(
            "confirmation_timeout_sec"
        ).get_parameter_value().double_value

        self.candidate_merge_radius = self.get_parameter(
            "candidate_merge_radius"
        ).get_parameter_value().double_value

        self.landmark_merge_radius = self.get_parameter(
            "landmark_merge_radius"
        ).get_parameter_value().double_value

        self.max_distance = self.get_parameter(
            "max_distance"
        ).get_parameter_value().double_value

        self.min_distance = self.get_parameter(
            "min_distance"
        ).get_parameter_value().double_value

        self.class_whitelist_csv = self.get_parameter(
            "class_whitelist_csv"
        ).get_parameter_value().string_value

        self.publish_markers = self.get_parameter(
            "publish_markers"
        ).get_parameter_value().bool_value

        self.class_whitelist = set()
        if self.class_whitelist_csv.strip():
            self.class_whitelist = {
                sanitize_name(c) for c in self.class_whitelist_csv.split(",")
            }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.candidates: List[Dict] = []
        self.landmarks: List[Dict] = []

        self.load_landmarks()

        self.sub = self.create_subscription(
            DetectionArray,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/yolo/landmarks",
            10,
        )

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f"Listening: {self.detections_topic}")
        self.get_logger().info(f"Saving landmarks in frame: {self.map_frame}")
        self.get_logger().info(f"Landmarks file: {self.landmarks_file}")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def load_landmarks(self):
        if not os.path.exists(self.landmarks_file):
            self.landmarks = []
            return

        try:
            with open(self.landmarks_file, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.landmarks = data.get("landmarks", [])
            self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks")

        except Exception as e:
            self.get_logger().warn(f"Could not load landmarks file: {e}")
            self.landmarks = []

    def save_landmarks(self):
        os.makedirs(os.path.dirname(self.landmarks_file), exist_ok=True)

        data = {
            "frame_id": self.map_frame,
            "landmarks": self.landmarks,
        }

        try:
            with open(self.landmarks_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)

        except Exception as e:
            self.get_logger().error(f"Could not save landmarks: {e}")

    def transform_point_to_map(
        self,
        source_frame: str,
        point: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float]]:
        if source_frame == "":
            source_frame = self.map_frame

        if source_frame == self.map_frame:
            return point

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
            )

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            rx, ry, rz = quat_rotate_vector(
                qx, qy, qz, qw,
                point[0], point[1], point[2],
            )

            return rx + tx, ry + ty, rz + tz

        except TransformException as e:
            self.get_logger().warn(
                f"Could not transform {source_frame} -> {self.map_frame}: {e}"
            )
            return None

    def is_valid_detection(self, det) -> bool:
        if det.score < self.min_score:
            return False

        class_name = sanitize_name(det.class_name)

        if self.class_whitelist and class_name not in self.class_whitelist:
            return False

        p = det.bbox3d.center.position
        x, y, z = p.x, p.y, p.z

        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            return False

        dist = math.sqrt(x * x + y * y + z * z)

        if dist < self.min_distance:
            return False

        if dist > self.max_distance:
            return False

        return True

    def update_candidate(
        self,
        class_name: str,
        position: Tuple[float, float, float],
        score: float,
    ):
        now = self.now_sec()

        best_candidate = None
        best_dist = float("inf")

        for cand in self.candidates:
            if cand["class_name"] != class_name:
                continue

            cand_pos = (cand["x"], cand["y"], cand["z"])
            d = distance_3d(position, cand_pos)

            if d < self.candidate_merge_radius and d < best_dist:
                best_candidate = cand
                best_dist = d

        if best_candidate is None:
            candidate = {
                "class_name": class_name,
                "x": position[0],
                "y": position[1],
                "z": position[2],
                "hits": 1,
                "score_sum": score,
                "first_seen": now,
                "last_seen": now,
                "promoted": False,
            }
            self.candidates.append(candidate)
            return

        hits = best_candidate["hits"]
        new_hits = hits + 1

        # Promedio incremental para estabilizar posición.
        best_candidate["x"] = (best_candidate["x"] * hits + position[0]) / new_hits
        best_candidate["y"] = (best_candidate["y"] * hits + position[1]) / new_hits
        best_candidate["z"] = (best_candidate["z"] * hits + position[2]) / new_hits
        best_candidate["hits"] = new_hits
        best_candidate["score_sum"] += score
        best_candidate["last_seen"] = now

        if (
            not best_candidate["promoted"]
            and best_candidate["hits"] >= self.min_confirmations
        ):
            self.promote_candidate(best_candidate)
            best_candidate["promoted"] = True

    def promote_candidate(self, candidate: Dict):
        class_name = candidate["class_name"]
        position = (candidate["x"], candidate["y"], candidate["z"])
        avg_score = candidate["score_sum"] / max(candidate["hits"], 1)

        # Evitar duplicados: si ya existe un landmark cercano de la misma clase, actualizarlo.
        for lm in self.landmarks:
            if lm["class_name"] != class_name:
                continue

            lm_pos = (lm["x"], lm["y"], lm["z"])
            d = distance_3d(position, lm_pos)

            if d < self.landmark_merge_radius:
                old_hits = lm.get("hits", 1)
                new_hits = old_hits + candidate["hits"]

                lm["x"] = (lm["x"] * old_hits + position[0] * candidate["hits"]) / new_hits
                lm["y"] = (lm["y"] * old_hits + position[1] * candidate["hits"]) / new_hits
                lm["z"] = (lm["z"] * old_hits + position[2] * candidate["hits"]) / new_hits
                lm["hits"] = new_hits
                lm["score"] = max(lm.get("score", 0.0), avg_score)
                lm["updated_at"] = self.now_sec()

                self.save_landmarks()

                self.get_logger().info(
                    f"Updated landmark: {class_name} "
                    f"x={lm['x']:.3f}, y={lm['y']:.3f}, z={lm['z']:.3f}"
                )
                return

        landmark_id = len(self.landmarks)

        landmark = {
            "id": landmark_id,
            "class_name": class_name,
            "frame_id": self.map_frame,
            "x": position[0],
            "y": position[1],
            "z": position[2],
            "score": avg_score,
            "hits": candidate["hits"],
            "created_at": self.now_sec(),
            "updated_at": self.now_sec(),
        }

        self.landmarks.append(landmark)
        self.save_landmarks()

        self.get_logger().info(
            f"Saved new landmark #{landmark_id}: {class_name} "
            f"x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}, "
            f"score={avg_score:.3f}, hits={candidate['hits']}"
        )

    def prune_old_candidates(self):
        now = self.now_sec()
        fresh_candidates = []

        for cand in self.candidates:
            age = now - cand["last_seen"]

            if age <= self.confirmation_timeout_sec:
                fresh_candidates.append(cand)

        self.candidates = fresh_candidates

    def detections_callback(self, msg: DetectionArray):
        for det in msg.detections:
            if not self.is_valid_detection(det):
                continue

            class_name = sanitize_name(det.class_name)

            source_frame = det.bbox3d.frame_id
            if source_frame == "":
                source_frame = msg.header.frame_id

            p = det.bbox3d.center.position
            raw_position = (p.x, p.y, p.z)

            map_position = self.transform_point_to_map(source_frame, raw_position)

            if map_position is None:
                continue

            self.update_candidate(class_name, map_position, det.score)

    def make_sphere_marker(self, lm: Dict, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_landmarks"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(lm["x"])
        marker.pose.position.y = float(lm["y"])
        marker.pose.position.z = float(lm["z"])
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 0.9

        return marker

    def make_text_marker(self, lm: Dict, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_landmark_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = float(lm["x"])
        marker.pose.position.y = float(lm["y"])
        marker.pose.position.z = float(lm["z"]) + 0.12
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.10

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"{lm['class_name']} #{lm['id']}"

        return marker

    def publish_landmark_markers(self):
        if not self.publish_markers:
            return

        marker_array = MarkerArray()

        marker_id = 0

        for lm in self.landmarks:
            marker_array.markers.append(self.make_sphere_marker(lm, marker_id))
            marker_id += 1

            marker_array.markers.append(self.make_text_marker(lm, marker_id))
            marker_id += 1

        self.marker_pub.publish(marker_array)

    def timer_callback(self):
        self.prune_old_candidates()
        self.publish_landmark_markers()


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()