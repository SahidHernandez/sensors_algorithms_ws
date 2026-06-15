import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


class DetectionLogger(Node):
    def __init__(self):
        super().__init__("detection_logger")

        self.declare_parameter("detections_topic", "detections")
        self.declare_parameter("report_period_sec", 2.0)

        self._detections_topic = self.get_parameter("detections_topic").value
        self._report_period_ns = int(
            float(self.get_parameter("report_period_sec").value) * 1e9
        )
        self._last_report_time_ns = 0

        self.create_subscription(
            DetectionArray,
            self._detections_topic,
            self._detections_callback,
            10,
        )

        self.get_logger().info(
            f"Listening for YOLO detections on '{self._detections_topic}'"
        )

    def _detections_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_report_time_ns < self._report_period_ns:
            return

        self._last_report_time_ns = now_ns

        class_names = sorted({detection.class_name for detection in msg.detections if detection.class_name})
        summary = ", ".join(class_names) if class_names else "no classes"

        self.get_logger().info(
            f"Frame with {len(msg.detections)} detections ({summary})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
