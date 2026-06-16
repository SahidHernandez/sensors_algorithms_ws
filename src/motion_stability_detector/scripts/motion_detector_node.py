#!/usr/bin/env python3
"""
Motion Detection and Stability Analysis Node.

Este nodo procesa flujos de video para detectar movimiento utilizando 
sustracción de fondo (MOG2/KNN) y análisis de estabilidad basado en el área 
de movimiento detectada.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge


class MotionDetectorNode(Node):
    """
    Nodo de ROS 2 que detecta movimiento mediante la sustracción de fondo.
    """
    def __init__(self):
        super().__init__('motion_detector_node')

        # -----------------------------
        # Parámetros
        # -----------------------------
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('algorithm', 'MOG2')  # MOG2 o KNN

        self.declare_parameter('history', 300)
        self.declare_parameter('var_threshold', 45.0)
        self.declare_parameter('dist2_threshold', 300.0)
        self.declare_parameter('detect_shadows', False)

        self.declare_parameter('blur_kernel', 5)
        self.declare_parameter('threshold_value', 200)
        self.declare_parameter('morph_iterations', 2)

        self.declare_parameter('min_contour_area', 300.0)
        self.declare_parameter('motion_threshold', 3000.0)

        self.declare_parameter('stable_frames_required', 5)
        self.declare_parameter('warmup_frames', 30)

        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_mask', True)

        self.declare_parameter('show_window', False)

        # -----------------------------
        # Cargar parámetros
        # -----------------------------
        self.image_topic = self.get_parameter('image_topic').value
        self.algorithm = self.get_parameter('algorithm').value.upper()

        self.history = int(self.get_parameter('history').value)
        self.var_threshold = float(self.get_parameter('var_threshold').value)
        self.dist2_threshold = float(self.get_parameter('dist2_threshold').value)
        self.detect_shadows = bool(self.get_parameter('detect_shadows').value)

        self.blur_kernel = int(self.get_parameter('blur_kernel').value)
        self.threshold_value = int(self.get_parameter('threshold_value').value)
        self.morph_iterations = int(self.get_parameter('morph_iterations').value)

        self.min_contour_area = float(self.get_parameter('min_contour_area').value)
        self.motion_threshold = float(self.get_parameter('motion_threshold').value)

        self.stable_frames_required = int(self.get_parameter('stable_frames_required').value)
        self.warmup_frames = int(self.get_parameter('warmup_frames').value)

        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.publish_mask = bool(self.get_parameter('publish_mask').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        # Asegurar kernel impar
        if self.blur_kernel < 1:
            self.blur_kernel = 1

        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1

        # -----------------------------
        # Bridge OpenCV - ROS
        # -----------------------------
        self.bridge = CvBridge()

        # -----------------------------
        # Crear sustractor de fondo
        # -----------------------------
        self.bg_subtractor = self.create_background_subtractor()

        # -----------------------------
        # Estado interno
        # -----------------------------
        self.frame_count = 0
        self.stable_counter = 0
        self.last_is_stable = False

        # -----------------------------
        # QoS para cámara
        # -----------------------------
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # -----------------------------
        # Subscriptores
        # -----------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos
        )

        # -----------------------------
        # Publicadores
        # -----------------------------
        self.motion_area_pub = self.create_publisher(
            Float32,
            '/motion_detector/motion_area',
            10
        )

        self.stable_pub = self.create_publisher(
            Bool,
            '/motion_detector/is_stable',
            10
        )

        self.mask_pub = self.create_publisher(
            Image,
            '/motion_detector/mask',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/motion_detector/debug_image',
            10
        )

        self.get_logger().info('Motion Detector Node iniciado')
        self.get_logger().info(f'Suscrito a: {self.image_topic}')
        self.get_logger().info(f'Algoritmo: {self.algorithm}')
        self.get_logger().info(f'Motion threshold: {self.motion_threshold}')
        self.get_logger().info(f'Stable frames required: {self.stable_frames_required}')

    def create_background_subtractor(self):
        """Inicializa el sustractor de fondo según el algoritmo seleccionado."""
        if self.algorithm == 'MOG2':
            self.get_logger().info('Usando BackgroundSubtractorMOG2')

            return cv2.createBackgroundSubtractorMOG2(
                history=self.history,
                varThreshold=self.var_threshold,
                detectShadows=self.detect_shadows
            )

        elif self.algorithm == 'KNN':
            self.get_logger().info('Usando BackgroundSubtractorKNN')

            return cv2.createBackgroundSubtractorKNN(
                history=self.history,
                dist2Threshold=self.dist2_threshold,
                detectShadows=self.detect_shadows
            )

        else:
            self.get_logger().warn(
                f'Algoritmo "{self.algorithm}" no reconocido. Usando MOG2 por defecto.'
            )

            return cv2.createBackgroundSubtractorMOG2(
                history=self.history,
                varThreshold=self.var_threshold,
                detectShadows=self.detect_shadows
            )

    def image_callback(self, msg):
        """
        Callback principal que procesa la imagen para detectar movimiento.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen ROS a OpenCV: {e}')
            return

        self.frame_count += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.blur_kernel > 1:
            gray = cv2.GaussianBlur(
                gray,
                (self.blur_kernel, self.blur_kernel),
                0
            )

        fg_mask = self.bg_subtractor.apply(gray)

        _, fg_mask = cv2.threshold(
            fg_mask,
            self.threshold_value,
            255,
            cv2.THRESH_BINARY
        )

        kernel = np.ones((3, 3), np.uint8)

        fg_mask = cv2.morphologyEx(
            fg_mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1
        )

        fg_mask = cv2.dilate(
            fg_mask,
            kernel,
            iterations=self.morph_iterations
        )

        contours, _ = cv2.findContours(
            fg_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        filtered_contours = []
        motion_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area >= self.min_contour_area:
                filtered_contours.append(contour)
                motion_area += area

        is_warmup = self.frame_count <= self.warmup_frames

        if is_warmup:
            is_stable = False
            self.stable_counter = 0
        else:
            if motion_area < self.motion_threshold:
                self.stable_counter += 1
            else:
                self.stable_counter = 0

            is_stable = self.stable_counter >= self.stable_frames_required

        self.last_is_stable = is_stable

        self.publish_results(
            frame=frame,
            mask=fg_mask,
            contours=filtered_contours,
            motion_area=motion_area,
            is_stable=is_stable,
            is_warmup=is_warmup
        )

    def publish_results(
        self,
        frame,
        mask,
        contours,
        motion_area,
        is_stable,
        is_warmup
    ):
        """Publica los resultados de la detección en los tópicos correspondientes."""
        motion_area_msg = Float32()
        motion_area_msg.data = float(motion_area)
        self.motion_area_pub.publish(motion_area_msg)

        stable_msg = Bool()
        stable_msg.data = bool(is_stable)
        self.stable_pub.publish(stable_msg)

        if self.publish_mask:
            try:
                mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                self.mask_pub.publish(mask_msg)
            except Exception as e:
                self.get_logger().error(f'Error publicando máscara: {e}')

        if self.publish_debug_image or self.show_window:
            debug_frame = frame.copy()

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(
                    debug_frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 255),
                    2
                )

            if is_warmup:
                status_text = 'WARMUP'
                status_color = (0, 255, 255)
            elif is_stable:
                status_text = 'ESTABLE'
                status_color = (0, 255, 0)
            else:
                status_text = 'MOVIMIENTO'
                status_color = (0, 0, 255)

            cv2.putText(
                debug_frame,
                f'Estado: {status_text}',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                status_color,
                2
            )

            cv2.putText(
                debug_frame,
                f'Motion area: {motion_area:.1f}',
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                status_color,
                2
            )

            cv2.putText(
                debug_frame,
                f'Stable counter: {self.stable_counter}/{self.stable_frames_required}',
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                status_color,
                2
            )

            if self.publish_debug_image:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(
                        debug_frame,
                        encoding='bgr8'
                    )
                    self.debug_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'Error publicando imagen debug: {e}')

            if self.show_window:
                cv2.imshow('Motion Detector Debug', debug_frame)
                cv2.imshow('Motion Mask', mask)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MotionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por usuario.')
    finally:
        node.destroy_node()
        if rclpy.ok():          # solo shutdown si el contexto sigue vivo
            rclpy.shutdown()


if __name__ == '__main__':
    main()