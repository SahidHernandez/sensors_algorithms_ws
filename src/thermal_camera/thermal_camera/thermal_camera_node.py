#!/usr/bin/env python3

import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


COLORMAPS = {
    'jet': cv2.COLORMAP_JET,
    'hot': cv2.COLORMAP_HOT,
    'magma': cv2.COLORMAP_MAGMA,
    'inferno': cv2.COLORMAP_INFERNO,
    'plasma': cv2.COLORMAP_PLASMA,
    'bone': cv2.COLORMAP_BONE,
    'spring': cv2.COLORMAP_SPRING,
    'autumn': cv2.COLORMAP_AUTUMN,
    'viridis': cv2.COLORMAP_VIRIDIS,
    'parula': getattr(cv2, 'COLORMAP_PARULA', cv2.COLORMAP_INFERNO),
    'rainbow': cv2.COLORMAP_RAINBOW,
}


class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')

        self.declare_parameter(
            'video_device',
            '/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0',
        )
        self.declare_parameter('image_topic', '/thermal_camera/image_raw')
        self.declare_parameter('frame_id', 'thermal_camera_link')
        self.declare_parameter('fps', 25.0)
        self.declare_parameter('scale', 3)
        self.declare_parameter('blur_radius', 0)
        self.declare_parameter('colormap', 'inferno')
        self.declare_parameter('publish_temperature_info', True)

        self.video_device = str(self.get_parameter('video_device').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.fps = max(1.0, float(self.get_parameter('fps').value))
        self.scale = max(1, int(self.get_parameter('scale').value))
        self.blur_radius = max(0, int(self.get_parameter('blur_radius').value))
        self.colormap_name = str(self.get_parameter('colormap').value).lower()
        self.publish_temperature_info = bool(self.get_parameter('publish_temperature_info').value)

        if self.colormap_name not in COLORMAPS:
            self.get_logger().warn(
                f'Colormap "{self.colormap_name}" no soportado. Usando inferno.'
            )
            self.colormap_name = 'inferno'

        self.capture: Optional[cv2.VideoCapture] = None
        self.last_reconnect_log_time = self.get_clock().now()

        # Frame compartido entre hilo de captura y timer
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_pub = self.create_publisher(Image, self.image_topic, image_qos)
        self.temperature_pub = self.create_publisher(String, '/thermal_camera/temperature_info', 10)

        # Hilo dedicado de captura: no bloquea el executor
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.get_logger().info(f'Thermal camera device: {self.video_device}')
        self.get_logger().info(f'Publishing thermal image on: {self.image_topic}')

    # ------------------------------------------------------------------
    # Hilo de captura
    # ------------------------------------------------------------------

    def _capture_loop(self):
        """Captura frames continuamente en hilo separado."""
        while rclpy.ok():
            if self.capture is None:
                if not self.open_capture():
                    self.log_reconnect_warning()
                    time.sleep(0.5)
                    continue

            ok, frame = self.capture.read()
            if not ok or frame is None:
                self.get_logger().warn('No se pudo leer frame termico. Reintentando apertura.')
                if self.capture is not None:
                    self.capture.release()
                    self.capture = None
                continue

            with self._frame_lock:
                self._latest_frame = frame

    def open_capture(self) -> bool:
        if self.capture is not None:
            self.capture.release()

        self.capture = cv2.VideoCapture(self.video_device, cv2.CAP_V4L)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = None
            return False

        self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
        return True

    # ------------------------------------------------------------------
    # Timer callback: solo procesa y publica
    # ------------------------------------------------------------------

    def timer_callback(self):
        with self._frame_lock:
            frame = self._latest_frame

        if frame is None:
            return

        try:
            heatmap, temperature_info = self.process_frame(frame)
        except Exception as exc:
            self.get_logger().error(f'Error procesando frame termico: {exc}')
            return

        self.image_pub.publish(self.cv_image_to_msg(heatmap))

        if self.publish_temperature_info:
            temp_msg = String()
            temp_msg.data = temperature_info
            self.temperature_pub.publish(temp_msg)

    def log_reconnect_warning(self):
        now = self.get_clock().now()
        if (now - self.last_reconnect_log_time).nanoseconds < int(2e9):
            return
        self.last_reconnect_log_time = now
        self.get_logger().warn(f'No se pudo abrir la camara termica en {self.video_device}')

    # ------------------------------------------------------------------
    # Procesamiento de frame
    # ------------------------------------------------------------------

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, str]:
        if frame.ndim != 3 or frame.shape[2] != 2:
            raise RuntimeError(f'Formato de frame inesperado: {frame.shape}')

        # Zero-copy views en lugar de array_split (evita copia)
        half = frame.shape[0] // 2
        image_data  = frame[:half]
        thermal_data = frame[half:]

        center_temp = self.pixel_to_celsius(thermal_data[96][128])
        min_position = np.unravel_index(np.argmin(thermal_data[..., 1]), thermal_data[..., 1].shape)
        max_position = np.unravel_index(np.argmax(thermal_data[..., 1]), thermal_data[..., 1].shape)
        min_temp = self.pixel_to_celsius(thermal_data[min_position])
        max_temp = self.pixel_to_celsius(thermal_data[max_position])
        avg_temp = self.average_temperature(thermal_data)

        # Aplica colormap directo sobre canal Y (evita cvtColor BGR completo)
        y_channel = np.ascontiguousarray(image_data[:, :, 0])
        y_resized = cv2.resize(
            y_channel,
            (image_data.shape[1] * self.scale, image_data.shape[0] * self.scale),
            interpolation=cv2.INTER_CUBIC,
        )

        if self.blur_radius > 0:
            y_resized = cv2.blur(y_resized, (self.blur_radius, self.blur_radius))

        heatmap = cv2.applyColorMap(y_resized, COLORMAPS[self.colormap_name])

        self.draw_crosshair(heatmap, center_temp)
        self.draw_extreme_marker(heatmap, max_position, max_temp, (0, 0, 255))
        self.draw_extreme_marker(heatmap, min_position, min_temp, (255, 0, 0))
        self.draw_hud(heatmap, avg_temp, center_temp, min_temp, max_temp)

        info = (
            f'center_c={center_temp:.2f};avg_c={avg_temp:.2f};'
            f'min_c={min_temp:.2f};max_c={max_temp:.2f}'
        )
        return heatmap, info

    @staticmethod
    def pixel_to_celsius(pixel: np.ndarray) -> float:
        hi = float(pixel[0])
        lo = float(pixel[1]) * 256.0
        return ((hi + lo) / 64.0) - 273.15

    def average_temperature(self, thermal_data: np.ndarray) -> float:
        low_mean = float(np.mean(thermal_data[..., 1])) * 256.0
        high_mean = float(np.mean(thermal_data[..., 0]))
        return ((high_mean + low_mean) / 64.0) - 273.15

    # ------------------------------------------------------------------
    # Dibujo HUD
    # ------------------------------------------------------------------

    def draw_crosshair(self, heatmap: np.ndarray, center_temp: float):
        height, width = heatmap.shape[:2]
        cx = width // 2
        cy = height // 2
        cv2.line(heatmap, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 2)
        cv2.line(heatmap, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 2)
        cv2.line(heatmap, (cx, cy - 20), (cx, cy + 20), (0, 0, 0), 1)
        cv2.line(heatmap, (cx - 20, cy), (cx + 20, cy), (0, 0, 0), 1)
        self.put_label(heatmap, f'{center_temp:.2f} C', (cx + 10, cy - 10))

    def draw_extreme_marker(
        self,
        heatmap: np.ndarray,
        pixel_position: Tuple[int, int],
        temperature: float,
        color: Tuple[int, int, int],
    ):
        row, col = pixel_position
        x = col * self.scale
        y = row * self.scale
        cv2.circle(heatmap, (x, y), 5, (0, 0, 0), 2)
        cv2.circle(heatmap, (x, y), 5, color, -1)
        self.put_label(heatmap, f'{temperature:.2f} C', (x + 10, y + 5))

    def draw_hud(
        self,
        heatmap: np.ndarray,
        avg_temp: float,
        center_temp: float,
        min_temp: float,
        max_temp: float,
    ):
        cv2.rectangle(heatmap, (0, 0), (220, 86), (0, 0, 0), -1)
        hud_lines = [
            f'Avg: {avg_temp:.2f} C',
            f'Center: {center_temp:.2f} C',
            f'Min: {min_temp:.2f} C',
            f'Max: {max_temp:.2f} C',
            f'Colormap: {self.colormap_name}',
        ]
        for index, line in enumerate(hud_lines):
            y = 16 + (index * 14)
            cv2.putText(
                heatmap, line, (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA,
            )

    @staticmethod
    def put_label(heatmap: np.ndarray, text: str, position: Tuple[int, int]):
        cv2.putText(heatmap, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(heatmap, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # Conversión a ROS Image
    # ------------------------------------------------------------------

    def cv_image_to_msg(self, image: np.ndarray) -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = image.shape[1] * image.shape[2]
        # Asegura array contiguo antes de serializar
        if not image.flags['C_CONTIGUOUS']:
            image = np.ascontiguousarray(image)
        msg.data = image.tobytes()
        return msg

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()