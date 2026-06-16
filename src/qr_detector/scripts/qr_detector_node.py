#!/usr/bin/env python3
"""Nodo ROS2 para detección de códigos QR en imágenes de cámara.

Utiliza pyzbar/zbar para la detección, aplica filtro de nitidez (sharpness)
para seleccionar capturas de calidad, y publica resultados en múltiples topics.
"""

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
    """Nodo ROS2 para detección de códigos QR con filtro de nitidez.

    Suscribe a un topic de imagen, detecta códigos QR usando pyzbar,
    calcula la nitidez del frame y publica resultados en múltiples topics.
    Opcionalmente publica imágenes de debug con anotaciones visuales y
    capturas automáticas cuando se cumplen condiciones de calidad.

    Publishers:
        /qr_detector/detected (Bool): True si se detectó al menos un QR.
        /qr_detector/text (String): Texto decodificado del QR (vacío si no hay).
        /qr_detector/debug_image (Image): Frame con anotaciones visuales.
        /qr_detector/sharpness (Float32): Valor de nitidez del frame actual.
        /qr_detector/capture_ready (Bool): True cuando se cumplen condiciones de captura.
        /qr_detector/capture_image (Image): Frame capturado (debug o limpio según config).
        /qr_detector/capture_text (String): Texto QR asociado a la captura.

    Subscribers:
        {image_topic} (Image): Topic de imagen configurable (default: /camera/color/image_raw).

    Parameters:
        image_topic (str): Topic de imagen de entrada.
        publish_debug_image (bool): Publicar imagen con anotaciones.
        show_window (bool): Mostrar ventana OpenCV local.
        resize_width (int): Ancho de redimensión para detección (0 = sin resize).
        use_grayscale (bool): Convertir a escala de grises antes de detectar.
        use_equalization (bool): Aplicar ecualización de histograma.
        use_adaptive_threshold (bool): Aplicar umbralización adaptativa.
        draw_qr_polygon (bool): Dibujar polígono del QR en el debug image.
        draw_qr_text (bool): Dibujar texto del QR en el debug image.
        print_only_when_changed (bool): Loggear QR solo cuando cambia el texto.
        min_text_length (int): Longitud mínima del texto QR para ser válido.
        sharpness_threshold (float): Umbral mínimo de nitidez para captura.
        capture_only_if_qr_detected (bool): Requerir QR detectado para capturar.
        capture_cooldown_frames (int): Frames mínimos entre capturas consecutivas.
        publish_capture_debug (bool): Publicar captura con anotaciones (True) o limpia (False).
    """

    def __init__(self):
        """Inicializa el nodo, declara parámetros, crea publishers y subscriber."""
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
        self.qr_detected_pub = self.create_publisher(Bool, '/qr_detector/detected', 10)
        self.qr_text_pub = self.create_publisher(String, '/qr_detector/text', 10)
        self.debug_image_pub = self.create_publisher(Image, '/qr_detector/debug_image', 10)
        self.sharpness_pub = self.create_publisher(Float32, '/qr_detector/sharpness', 10)
        self.capture_ready_pub = self.create_publisher(Bool, '/qr_detector/capture_ready', 10)
        self.capture_image_pub = self.create_publisher(Image, '/qr_detector/capture_image', 10)
        self.capture_text_pub = self.create_publisher(String, '/qr_detector/capture_text', 10)

        self.get_logger().info('QR Detector Node iniciado con pyzbar/zbar + sharpness filter')
        self.get_logger().info(f'Suscrito a: {self.image_topic}')
        self.get_logger().info(f'Sharpness threshold: {self.sharpness_threshold}')

    # -------------------------------------------------------------------------
    # Sharpness
    # -------------------------------------------------------------------------
    def calculate_sharpness(self, frame: np.ndarray) -> float:
        """Calcula la nitidez de un frame usando la varianza del Laplaciano.

        Convierte el frame a escala de grises y aplica el operador Laplaciano.
        Una varianza alta indica imagen nítida; baja indica imagen borrosa.

        Args:
            frame: Frame BGR de entrada como array NumPy (H, W, 3).

        Returns:
            Varianza del Laplaciano como medida de nitidez. Valores más altos
            indican mayor nitidez.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
        return float(sharpness)

    # -------------------------------------------------------------------------
    # Preprocesamiento QR
    # -------------------------------------------------------------------------
    def preprocess_image(self, frame: np.ndarray) -> np.ndarray:
        """Aplica preprocesamiento al frame para mejorar la detección de QR.

        Ejecuta en orden (según parámetros activos): conversión a escala de
        grises, ecualización de histograma y umbralización adaptativa gaussiana.

        Args:
            frame: Frame BGR de entrada como array NumPy.

        Returns:
            Frame procesado. Puede ser escala de grises (H, W) o BGR (H, W, 3)
            dependiendo de los parámetros configurados.
        """
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
                processed, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31, 5
            )

        return processed

    # -------------------------------------------------------------------------
    # Detección QR
    # -------------------------------------------------------------------------
    def detect_qr(self, frame: np.ndarray) -> tuple[bool, str, list[dict]]:
        """Detecta códigos QR en el frame usando pyzbar.

        Opcionalmente redimensiona el frame antes de la detección y aplica
        preprocesamiento. Filtra resultados por longitud mínima de texto.
        Las coordenadas del polígono y rect se escalan de vuelta al tamaño
        original del frame.

        Args:
            frame: Frame BGR de entrada como array NumPy.

        Returns:
            Tuple de tres elementos:

            - detected (bool): True si se detectó al menos un QR válido.
            - final_text (str): Textos detectados concatenados con " | ".
              Vacío si no se detectó ninguno.
            - qr_draw_data (list[dict]): Lista de dicts con datos para dibujar
              cada QR. Cada dict contiene:

              - ``'text'`` (str): Texto decodificado.
              - ``'polygon'`` (list): Puntos del polígono en coordenadas del
                frame redimensionado.
              - ``'rect'`` (Rect): Bounding rect en coordenadas del frame
                redimensionado.
              - ``'scale_factor'`` (float): Factor usado para escalar de vuelta.
        """
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
        image: np.ndarray,
        text: str,
        x: int,
        y: int,
        font_scale: float = 0.7,
        text_color: tuple = (255, 255, 255),
        box_color: tuple = (0, 120, 0),
        thickness: int = 2,
        padding: int = 7
    ) -> np.ndarray:
        """Dibuja un texto con caja de fondo de color sobre una imagen.

        Calcula el tamaño del texto, ajusta la posición para evitar salirse
        de los bordes de la imagen y dibuja el rectángulo relleno antes del texto.

        Args:
            image: Frame BGR sobre el cual dibujar (modificado in-place).
            text: Cadena de texto a renderizar.
            x: Coordenada X de la esquina superior izquierda de la caja.
            y: Coordenada Y de la línea base del texto.
            font_scale: Escala de la fuente OpenCV. Defaults to 0.7.
            text_color: Color BGR del texto. Defaults to (255, 255, 255).
            box_color: Color BGR del rectángulo de fondo. Defaults to (0, 120, 0).
            thickness: Grosor del trazo del texto. Defaults to 2.
            padding: Padding en píxeles entre el texto y el borde de la caja.
                Defaults to 7.

        Returns:
            Frame con la caja de texto dibujada (mismo objeto que ``image``).
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size, baseline = cv2.getTextSize(text, font, font_scale, thickness)
        text_w, text_h = text_size
        img_h, img_w = image.shape[:2]

        x = max(5, min(x, img_w - text_w - padding * 2 - 5))
        y = max(text_h + padding * 2 + 5, min(y, img_h - 5))

        box_x1 = x
        box_y1 = y - text_h - padding * 2
        box_x2 = x + text_w + padding * 2
        box_y2 = y + baseline

        cv2.rectangle(image, (box_x1, box_y1), (box_x2, box_y2), box_color, -1)
        cv2.putText(
            image, text, (x + padding, y - padding),
            font, font_scale, text_color, thickness, cv2.LINE_AA
        )

        return image

    # -------------------------------------------------------------------------
    # Dibujar resultados QR
    # -------------------------------------------------------------------------
    def draw_qr_results(self, debug_frame: np.ndarray, qr_draw_data: list[dict]) -> np.ndarray:
        """Dibuja polígonos, vértices y texto de todos los QR detectados.

        Para cada QR en ``qr_draw_data``, escala las coordenadas al tamaño
        original del frame y dibuja (según parámetros):

        - Polígono verde con puntos de vértice rojos, o bounding rect si el
          polígono tiene menos de 4 puntos.
        - Caja de texto con el contenido del QR (truncado a 35 caracteres).
          Se posiciona arriba del QR si hay espacio, o abajo en caso contrario.

        Args:
            debug_frame: Frame BGR de destino (modificado in-place).
            qr_draw_data: Lista de dicts generada por :meth:`detect_qr`.

        Returns:
            Frame con todas las anotaciones de QR dibujadas (mismo objeto
            que ``debug_frame``).
        """
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

            if self.draw_qr_polygon:
                if polygon is not None and len(polygon) > 0:
                    pts = [[int(p.x / scale_factor), int(p.y / scale_factor)] for p in polygon]
                    pts = np.array(pts, dtype=np.int32)

                    if len(pts) >= 4:
                        cv2.polylines(debug_frame, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
                        for pt in pts:
                            cv2.circle(debug_frame, tuple(pt), 5, (0, 0, 255), -1)
                    else:
                        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                else:
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            if self.draw_qr_text:
                display_text = qr_text if len(qr_text) <= 35 else qr_text[:35] + "..."
                text_x = x
                text_y = y - 10 if y > 45 else y + h + 35

                self.draw_text_box(
                    debug_frame, f"Text: {display_text}", text_x, text_y,
                    font_scale=0.65, text_color=(255, 255, 255),
                    box_color=(0, 120, 0), thickness=2
                )

        return debug_frame

    # -------------------------------------------------------------------------
    # Condición para publicar captura
    # -------------------------------------------------------------------------
    def should_publish_capture(self, detected: bool, sharpness: float) -> bool:
        """Evalúa si se deben cumplir las condiciones para publicar una captura.

        Verifica tres condiciones en orden:

        1. Nitidez del frame >= ``sharpness_threshold``.
        2. Si ``capture_only_if_qr_detected`` es True, además requiere QR detectado.
        3. Han pasado al menos ``capture_cooldown_frames`` frames desde la última captura.

        Args:
            detected: True si se detectó al menos un QR en el frame actual.
            sharpness: Valor de nitidez calculado para el frame actual.

        Returns:
            True si todas las condiciones se cumplen y se debe publicar captura.
        """
        sharpness_ok = sharpness >= self.sharpness_threshold

        if self.capture_only_if_qr_detected:
            base_condition = detected and sharpness_ok
        else:
            base_condition = sharpness_ok

        cooldown_ok = (self.frame_counter - self.last_capture_frame) >= self.capture_cooldown_frames

        return base_condition and cooldown_ok

    # -------------------------------------------------------------------------
    # Publicar captura
    # -------------------------------------------------------------------------
    def publish_capture(self, frame: np.ndarray, debug_frame: np.ndarray, qr_text: str) -> None:
        """Publica la captura seleccionada y su texto QR asociado.

        Selecciona entre ``debug_frame`` (con anotaciones) o ``frame`` (limpio)
        según ``publish_capture_debug``. Actualiza ``last_capture_frame`` para
        el control de cooldown.

        Publica en:
            - ``/qr_detector/capture_image``: Frame seleccionado como Image BGR.
            - ``/qr_detector/capture_text``: Texto QR como String.

        Args:
            frame: Frame BGR original sin anotaciones.
            debug_frame: Frame BGR con anotaciones visuales.
            qr_text: Texto del QR detectado a publicar junto con la captura.
        """
        try:
            capture_frame = debug_frame if self.publish_capture_debug else frame
            capture_msg = self.bridge.cv2_to_imgmsg(capture_frame, encoding='bgr8')
            self.capture_image_pub.publish(capture_msg)

            text_msg = String()
            text_msg.data = qr_text
            self.capture_text_pub.publish(text_msg)

            self.last_capture_frame = self.frame_counter
            self.get_logger().info(f'Captura publicada | QR: "{qr_text}"')

        except Exception as e:
            self.get_logger().error(f'Error publicando captura: {e}')

    # -------------------------------------------------------------------------
    # Callback principal
    # -------------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        """Callback principal del subscriber de imagen.

        Ejecuta el pipeline completo por cada frame recibido:

        1. Convierte el mensaje ROS ``Image`` a array BGR con CvBridge.
        2. Calcula y publica la nitidez del frame.
        3. Detecta QR y publica estado (detected, text).
        4. Genera el debug frame con anotaciones y overlay de estado.
        5. Evalúa condición de captura y publica si corresponde.
        6. Publica el debug image si ``publish_debug_image`` está activo.
        7. Muestra ventana OpenCV local si ``show_window`` está activo.

        Args:
            msg: Mensaje ``sensor_msgs/Image`` recibido desde el topic de cámara.
        """
        self.frame_counter += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen ROS a OpenCV: {e}')
            return

        # Calcular y publicar sharpness
        sharpness = self.calculate_sharpness(frame)
        sharpness_msg = Float32()
        sharpness_msg.data = float(sharpness)
        self.sharpness_pub.publish(sharpness_msg)

        # Detectar QR y publicar resultados
        detected, qr_text, qr_draw_data = self.detect_qr(frame)

        detected_msg = Bool()
        detected_msg.data = bool(detected)
        self.qr_detected_pub.publish(detected_msg)

        text_msg = String()
        text_msg.data = qr_text if detected else ""
        self.qr_text_pub.publish(text_msg)

        # Generar debug frame con anotaciones
        debug_frame = frame.copy()
        debug_frame = self.draw_qr_results(debug_frame, qr_draw_data)

        status_text = "QR Detected" if detected else "No QR"
        status_box_color = (0, 120, 0) if detected else (0, 0, 180)

        self.draw_text_box(
            debug_frame, status_text, 20, 45,
            font_scale=0.75, text_color=(255, 255, 255),
            box_color=status_box_color, thickness=2
        )

        if detected:
            display_text = qr_text if len(qr_text) <= 45 else qr_text[:45] + "..."
            self.draw_text_box(
                debug_frame, f"Text: {display_text}", 20, 90,
                font_scale=0.65, text_color=(255, 255, 255),
                box_color=(0, 120, 0), thickness=2
            )

            if self.print_only_when_changed:
                if qr_text != self.last_text:
                    self.get_logger().info(f'QR detectado: {qr_text}')
                    self.last_text = qr_text
            else:
                self.get_logger().info(f'QR detectado: {qr_text}')

        # Evaluar y publicar captura
        capture_ready = self.should_publish_capture(detected, sharpness)

        capture_ready_msg = Bool()
        capture_ready_msg.data = bool(capture_ready)
        self.capture_ready_pub.publish(capture_ready_msg)

        if capture_ready:
            self.publish_capture(frame=frame, debug_frame=debug_frame, qr_text=qr_text)

        # Publicar debug image
        if self.publish_debug_image:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Error publicando imagen debug: {e}')

        # Ventana local opcional
        if self.show_window:
            cv2.imshow('QR Detector Debug', debug_frame)
            cv2.waitKey(1)


def main(args=None):
    """Punto de entrada del nodo QR Detector.

    Inicializa rclpy, instancia :class:`QRDetectorNode` y lo mantiene
    activo con ``rclpy.spin``. Destruye el nodo y cierra ventanas OpenCV
    al recibir ``KeyboardInterrupt``.

    Args:
        args: Argumentos de línea de comandos pasados a ``rclpy.init``.
            Defaults to None (usa ``sys.argv``).
    """
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