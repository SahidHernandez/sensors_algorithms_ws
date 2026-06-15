#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from cv_bridge import CvBridge

from pyzbar import pyzbar


class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')    

        # -----------------------------
        # Parámetros generales
        # -----------------------------
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_window', False)

        # -----------------------------
        # Parámetros QR
        # -----------------------------
        self.declare_parameter('resize_width', 0)
        self.declare_parameter('use_grayscale', True)
        self.declare_parameter('use_equalization', False)
        self.declare_parameter('use_adaptive_threshold', False)

        self.declare_parameter('draw_qr_polygon', True)
        self.declare_parameter('draw_qr_text', True)
        self.declare_parameter('print_only_when_changed', True)
        self.declare_parameter('min_text_length', 1)

        # -----------------------------
        # Parámetros sharpness / captura
        # -----------------------------
        self.declare_parameter('sharpness_threshold', 350.0)
        self.declare_parameter('capture_only_if_qr_detected', True)
        self.declare_parameter('capture_cooldown_frames', 30)

        # true = publica captura con cuadro y texto
        # false = publica captura limpia
        self.declare_parameter('publish_capture_debug', True)

        # -----------------------------
        # Leer parámetros
        # -----------------------------
        self.image_topic = self.get_parameter('image_topic').value
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        self.resize_width = int(self.get_parameter('resize_width').value)
        self.use_grayscale = bool(self.get_parameter('use_grayscale').value)
        self.use_equalization = bool(self.get_parameter('use_equalization').value)
        self.use_adaptive_threshold = bool(self.get_parameter('use_adaptive_threshold').value)

        self.draw_qr_polygon = bool(self.get_parameter('draw_qr_polygon').value)
        self.draw_qr_text = bool(self.get_parameter('draw_qr_text').value)
        self.print_only_when_changed = bool(self.get_parameter('print_only_when_changed').value)
        self.min_text_length = int(self.get_parameter('min_text_length').value)

        self.sharpness_threshold = float(self.get_parameter('sharpness_threshold').value)
        self.capture_only_if_qr_detected = bool(self.get_parameter('capture_only_if_qr_detected').value)
        self.capture_cooldown_frames = int(self.get_parameter('capture_cooldown_frames').value)
        self.publish_capture_debug = bool(self.get_parameter('publish_capture_debug').value)

        # -----------------------------
        # Estado interno
        # -----------------------------
        self.bridge = CvBridge()
        self.last_text = ""
        self.frame_counter = 0
        self.last_capture_frame = -999999

        # -----------------------------
        # QoS para cámara
        # -----------------------------
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # -----------------------------
        # Subscriber
        # -----------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos
        )

        # -----------------------------
        # Publishers
        # -----------------------------
        self.qr_detected_pub = self.create_publisher(
            Bool,
            '/qr_detector/detected',
            10
        )

        self.qr_text_pub = self.create_publisher(
            String,
            '/qr_detector/text',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/qr_detector/debug_image',
            10
        )

        self.sharpness_pub = self.create_publisher(
            Float32,
            '/qr_detector/sharpness',
            10
        )

        self.capture_ready_pub = self.create_publisher(
            Bool,
            '/qr_detector/capture_ready',
            10
        )

        self.capture_image_pub = self.create_publisher(
            Image,
            '/qr_detector/capture_image',
            10
        )

        self.capture_text_pub = self.create_publisher(
            String,
            '/qr_detector/capture_text',
            10
        )

        self.get_logger().info('QR Detector Node iniciado con pyzbar/zbar + sharpness filter')
        self.get_logger().info(f'Suscrito a: {self.image_topic}')
        self.get_logger().info(f'Sharpness threshold: {self.sharpness_threshold}')

    # -------------------------------------------------------------------------
    # Sharpness
    # -------------------------------------------------------------------------
    def calculate_sharpness(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
        return float(sharpness)

    # -------------------------------------------------------------------------
    # Preprocesamiento QR
    # -------------------------------------------------------------------------
    def preprocess_image(self, frame):
        processed = frame.copy()

        if self.use_grayscale:
            processed = cv2.cvtColor(processed, cv2.COLOR_BGR2GRAY)

        if self.use_equalization:
            if len(processed.shape) == 2:
                processed = cv2.equalizeHist(processed)
            else:
                gray = cv2.cvtColor(processed, cv2.COLOR_BGR2GRAY)
                processed = cv2.equalizeHist(gray)

        if self.use_adaptive_threshold:
            if len(processed.shape) == 3:
                processed = cv2.cvtColor(processed, cv2.COLOR_BGR2GRAY)

            processed = cv2.adaptiveThreshold(
                processed,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                5
            )

        return processed

    # -------------------------------------------------------------------------
    # Detección QR
    # -------------------------------------------------------------------------
    def detect_qr(self, frame):
        detection_frame = frame.copy()
        scale_factor = 1.0

        if self.resize_width > 0:
            h, w = detection_frame.shape[:2]

            if w > 0:
                scale_factor = self.resize_width / float(w)
                new_height = int(h * scale_factor)

                detection_frame = cv2.resize(
                    detection_frame,
                    (self.resize_width, new_height),
                    interpolation=cv2.INTER_AREA
                )

        processed_frame = self.preprocess_image(detection_frame)

        qr_codes = pyzbar.decode(processed_frame)

        detected_texts = []
        qr_draw_data = []

        for qr in qr_codes:
            try:
                qr_text = qr.data.decode('utf-8', errors='replace').strip()
            except Exception:
                qr_text = str(qr.data).strip()

            if len(qr_text) < self.min_text_length:
                continue

            detected_texts.append(qr_text)

            qr_draw_data.append({
                'text': qr_text,
                'polygon': qr.polygon,
                'rect': qr.rect,
                'scale_factor': scale_factor
            })

        detected = len(detected_texts) > 0
        final_text = " | ".join(detected_texts)

        return detected, final_text, qr_draw_data

    # -------------------------------------------------------------------------
    # Dibujar caja de texto
    # -------------------------------------------------------------------------
    def draw_text_box(
        self,
        image,
        text,
        x,
        y,
        font_scale=0.7,
        text_color=(255, 255, 255),
        box_color=(0, 120, 0),
        thickness=2,
        padding=7
    ):
        font = cv2.FONT_HERSHEY_SIMPLEX

        text_size, baseline = cv2.getTextSize(
            text,
            font,
            font_scale,
            thickness
        )

        text_w, text_h = text_size
        img_h, img_w = image.shape[:2]

        x = max(5, min(x, img_w - text_w - padding * 2 - 5))
        y = max(text_h + padding * 2 + 5, min(y, img_h - 5))

        box_x1 = x
        box_y1 = y - text_h - padding * 2
        box_x2 = x + text_w + padding * 2
        box_y2 = y + baseline

        cv2.rectangle(
            image,
            (box_x1, box_y1),
            (box_x2, box_y2),
            box_color,
            -1
        )

        cv2.putText(
            image,
            text,
            (x + padding, y - padding),
            font,
            font_scale,
            text_color,
            thickness,
            cv2.LINE_AA
        )

        return image

    # -------------------------------------------------------------------------
    # Dibujar resultados QR
    # -------------------------------------------------------------------------
    def draw_qr_results(self, debug_frame, qr_draw_data):
        for qr_data in qr_draw_data:
            qr_text = qr_data['text']
            polygon = qr_data['polygon']
            rect = qr_data['rect']
            scale_factor = qr_data['scale_factor']

            x, y, w, h = rect
            x = int(x / scale_factor)
            y = int(y / scale_factor)
            w = int(w / scale_factor)
            h = int(h / scale_factor)

            # -----------------------------
            # Dibujar polígono del QR
            # -----------------------------
            if self.draw_qr_polygon:
                if polygon is not None and len(polygon) > 0:
                    pts = []

                    for p in polygon:
                        px = int(p.x / scale_factor)
                        py = int(p.y / scale_factor)
                        pts.append([px, py])

                    pts = np.array(pts, dtype=np.int32)

                    if len(pts) >= 4:
                        cv2.polylines(
                            debug_frame,
                            [pts],
                            isClosed=True,
                            color=(0, 255, 0),
                            thickness=3
                        )

                        for pt in pts:
                            cv2.circle(
                                debug_frame,
                                tuple(pt),
                                5,
                                (0, 0, 255),
                                -1
                            )
                    else:
                        cv2.rectangle(
                            debug_frame,
                            (x, y),
                            (x + w, y + h),
                            (0, 255, 0),
                            3
                        )
                else:
                    cv2.rectangle(
                        debug_frame,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        3
                    )

            # -----------------------------
            # Dibujar texto del QR
            # -----------------------------
            if self.draw_qr_text:
                display_text = qr_text

                if len(display_text) > 35:
                    display_text = display_text[:35] + "..."

                # Si hay espacio arriba, texto arriba.
                # Si no, texto abajo.
                if y > 45:
                    text_x = x
                    text_y = y - 10
                else:
                    text_x = x
                    text_y = y + h + 35

                self.draw_text_box(
                    debug_frame,
                    f"Text: {display_text}",
                    text_x,
                    text_y,
                    font_scale=0.65,
                    text_color=(255, 255, 255),
                    box_color=(0, 120, 0),
                    thickness=2
                )

        return debug_frame

    # -------------------------------------------------------------------------
    # Condición para publicar captura
    # -------------------------------------------------------------------------
    def should_publish_capture(self, detected, sharpness):
        sharpness_ok = sharpness >= self.sharpness_threshold

        if self.capture_only_if_qr_detected:
            base_condition = detected and sharpness_ok
        else:
            base_condition = sharpness_ok

        cooldown_ok = (
            self.frame_counter - self.last_capture_frame
        ) >= self.capture_cooldown_frames

        return base_condition and cooldown_ok

    # -------------------------------------------------------------------------
    # Publicar captura
    # -------------------------------------------------------------------------
    def publish_capture(self, frame, debug_frame, qr_text):
        try:
            if self.publish_capture_debug:
                capture_frame = debug_frame
            else:
                capture_frame = frame

            capture_msg = self.bridge.cv2_to_imgmsg(
                capture_frame,
                encoding='bgr8'
            )

            self.capture_image_pub.publish(capture_msg)

            text_msg = String()
            text_msg.data = qr_text
            self.capture_text_pub.publish(text_msg)

            self.last_capture_frame = self.frame_counter

            self.get_logger().info(
                f'Captura publicada | QR: "{qr_text}"'
            )

        except Exception as e:
            self.get_logger().error(f'Error publicando captura: {e}')

    # -------------------------------------------------------------------------
    # Callback principal
    # -------------------------------------------------------------------------
    def image_callback(self, msg):
        self.frame_counter += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen ROS a OpenCV: {e}')
            return

        # -----------------------------
        # Calcular sharpness
        # -----------------------------
        sharpness = self.calculate_sharpness(frame)

        sharpness_msg = Float32()
        sharpness_msg.data = float(sharpness)
        self.sharpness_pub.publish(sharpness_msg)

        # -----------------------------
        # Detectar QR
        # -----------------------------
        detected, qr_text, qr_draw_data = self.detect_qr(frame)

        detected_msg = Bool()
        detected_msg.data = bool(detected)
        self.qr_detected_pub.publish(detected_msg)

        text_msg = String()
        text_msg.data = qr_text if detected else ""
        self.qr_text_pub.publish(text_msg)

        # -----------------------------
        # Imagen debug
        # -----------------------------
        debug_frame = frame.copy()
        debug_frame = self.draw_qr_results(debug_frame, qr_draw_data)

        if detected:
            status_text = "QR Detected"
            status_box_color = (0, 120, 0)
        else:
            status_text = "No QR"
            status_box_color = (0, 0, 180)

        self.draw_text_box(
            debug_frame,
            status_text,
            20,
            45,
            font_scale=0.75,
            text_color=(255, 255, 255),
            box_color=status_box_color,
            thickness=2
        )

        if detected:
            display_text = qr_text

            if len(display_text) > 45:
                display_text = display_text[:45] + "..."

            self.draw_text_box(
                debug_frame,
                f"Text: {display_text}",
                20,
                90,
                font_scale=0.65,
                text_color=(255, 255, 255),
                box_color=(0, 120, 0),
                thickness=2
            )

            if self.print_only_when_changed:
                if qr_text != self.last_text:
                    self.get_logger().info(f'QR detectado: {qr_text}')
                    self.last_text = qr_text
            else:
                self.get_logger().info(f'QR detectado: {qr_text}')

        # -----------------------------
        # Captura
        # -----------------------------
        capture_ready = self.should_publish_capture(detected, sharpness)

        capture_ready_msg = Bool()
        capture_ready_msg.data = bool(capture_ready)
        self.capture_ready_pub.publish(capture_ready_msg)

        if capture_ready:
            self.publish_capture(
                frame=frame,
                debug_frame=debug_frame,
                qr_text=qr_text
            )

        # -----------------------------
        # Publicar debug image
        # -----------------------------
        if self.publish_debug_image:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(
                    debug_frame,
                    encoding='bgr8'
                )
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Error publicando imagen debug: {e}')

        # -----------------------------
        # Ventana OpenCV opcional
        # -----------------------------
        if self.show_window:
            cv2.imshow('QR Detector Debug', debug_frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = QRDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por usuario.')
    finally:
        if node.show_window:
            cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()