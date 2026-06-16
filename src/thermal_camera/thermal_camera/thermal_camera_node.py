#!/usr/bin/env python3
"""Nodo ROS2 para cámara térmica TC001 Max con colormap y HUD de temperatura.

Captura frames RAW YUV de la cámara térmica vía V4L2, extrae datos de imagen
y temperatura, aplica colormap configurable y publica el heatmap resultante
junto con información de temperatura en topics ROS2.

La captura corre en un hilo dedicado para no bloquear el executor de ROS2.
La publicación se controla mediante un timer a FPS configurable.
"""

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
    'jet':     cv2.COLORMAP_JET,
    'hot':     cv2.COLORMAP_HOT,
    'magma':   cv2.COLORMAP_MAGMA,
    'inferno': cv2.COLORMAP_INFERNO,
    'plasma':  cv2.COLORMAP_PLASMA,
    'bone':    cv2.COLORMAP_BONE,
    'spring':  cv2.COLORMAP_SPRING,
    'autumn':  cv2.COLORMAP_AUTUMN,
    'viridis': cv2.COLORMAP_VIRIDIS,
    'parula':  getattr(cv2, 'COLORMAP_PARULA', cv2.COLORMAP_INFERNO),
    'rainbow': cv2.COLORMAP_RAINBOW,
}
"""dict[str, int]: Mapa de nombres de colormap a constantes OpenCV.

``parula`` usa fallback a ``inferno`` si la versión de OpenCV no lo soporta.
Colormaps disponibles: jet, hot, magma, inferno, plasma, bone,
spring, autumn, viridis, parula, rainbow.
"""


class ThermalCameraNode(Node):
    """Nodo ROS2 para adquisición y publicación de imágenes de cámara térmica.

    Abre el dispositivo V4L2 de la cámara térmica, lee frames RAW en formato
    YUV de dos mitades (imagen + datos térmicos), genera un heatmap con
    colormap OpenCV y publica la imagen procesada junto con métricas de
    temperatura (centro, mínima, máxima, promedio).

    La captura de frames corre en un ``threading.Thread`` dedicado para evitar
    bloquear el executor de ROS2. Un ``Timer`` separado controla la publicación
    al FPS configurado, consumiendo el último frame disponible.

    Publishers:
        {image_topic} (sensor_msgs/Image): Heatmap BGR con HUD superpuesto.
        /thermal_camera/temperature_info (std_msgs/String): Métricas de
            temperatura en formato ``key=value`` separado por ``;``.

    Parameters:
        video_device (str): Ruta al dispositivo V4L2 de la cámara térmica.
            Default: ``/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0``.
        image_topic (str): Topic donde se publica el heatmap.
            Default: ``/thermal_camera/image_raw``.
        frame_id (str): Frame ID del header de los mensajes Image.
            Default: ``thermal_camera_link``.
        fps (float): Frecuencia de publicación en Hz (mínimo 1.0).
            Default: ``25.0``.
        scale (int): Factor de escala de la imagen de salida (mínimo 1).
            Default: ``3``.
        blur_radius (int): Radio del filtro de blur promedio en píxeles.
            ``0`` desactiva el blur. Default: ``0``.
        colormap (str): Nombre del colormap a aplicar (ver ``COLORMAPS``).
            Default: ``inferno``.
        publish_temperature_info (bool): Publicar métricas de temperatura en
            ``/thermal_camera/temperature_info``. Default: ``True``.
    """

    def __init__(self):
        """Inicializa el nodo, parámetros, hilo de captura, timer y publishers."""
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

        # Frame compartido entre hilo de captura y timer de publicación.
        # Protegido por _frame_lock para acceso thread-safe.
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_pub = self.create_publisher(Image, self.image_topic, image_qos)
        self.temperature_pub = self.create_publisher(
            String, '/thermal_camera/temperature_info', 10
        )

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.get_logger().info(f'Thermal camera device: {self.video_device}')
        self.get_logger().info(f'Publishing thermal image on: {self.image_topic}')

    # ------------------------------------------------------------------
    # Hilo de captura
    # ------------------------------------------------------------------

    def _capture_loop(self) -> None:
        """Bucle de captura continua de frames en hilo dedicado.

        Intenta abrir el dispositivo si no está disponible, reintentando
        cada 0.5 s con log throttled a 2 s. En cada iteración lee un frame;
        si la lectura falla libera el ``VideoCapture`` y reintenta la apertura
        en la siguiente iteración. El frame leído se almacena en
        ``_latest_frame`` bajo ``_frame_lock``.

        El bucle termina cuando ``rclpy.ok()`` retorna ``False``.
        """
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
        """Abre el dispositivo V4L2 de la cámara térmica.

        Libera cualquier ``VideoCapture`` previo antes de intentar la apertura.
        Configura ``CAP_PROP_CONVERT_RGB`` en ``0.0`` para recibir el frame
        RAW sin conversión de color automática.

        Returns:
            ``True`` si el dispositivo se abrió correctamente, ``False`` en
            caso contrario (el objeto ``capture`` queda en ``None``).
        """
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
    # Timer callback
    # ------------------------------------------------------------------

    def timer_callback(self) -> None:
        """Callback del timer de publicación.

        Lee el último frame disponible bajo ``_frame_lock``, lo procesa con
        :meth:`process_frame` y publica el heatmap en ``image_topic``.
        Si ``publish_temperature_info`` está activo, también publica las
        métricas en ``/thermal_camera/temperature_info``.

        No bloquea si no hay frame disponible aún (``_latest_frame`` es None).
        Loggea errores de procesamiento sin detener el nodo.
        """
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

    def log_reconnect_warning(self) -> None:
        """Loggea una advertencia de reconexión con throttle de 2 segundos.

        Evita spam en consola durante el bucle de reconexión comparando el
        tiempo actual contra ``last_reconnect_log_time``. Solo emite el log
        si han pasado al menos 2 s desde el último mensaje.
        """
        now = self.get_clock().now()
        if (now - self.last_reconnect_log_time).nanoseconds < int(2e9):
            return
        self.last_reconnect_log_time = now
        self.get_logger().warn(f'No se pudo abrir la camara termica en {self.video_device}')

    # ------------------------------------------------------------------
    # Procesamiento de frame
    # ------------------------------------------------------------------

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, str]:
        """Procesa un frame RAW de la cámara térmica y genera el heatmap.

        El frame RAW tiene dos mitades verticales:

        - **Superior** (``image_data``): datos de imagen canal Y.
        - **Inferior** (``thermal_data``): datos de temperatura en dos canales
          (byte alto y byte bajo por píxel).

        Pipeline:

        1. Valida formato (3 dims, 2 canales).
        2. Extrae temperatura de centro, mínima, máxima y promedio.
        3. Redimensiona el canal Y según ``scale`` con interpolación cúbica.
        4. Aplica blur promedio si ``blur_radius > 0``.
        5. Aplica colormap OpenCV.
        6. Superpone HUD con métricas y marcadores de extremos.

        Args:
            frame: Array NumPy con shape ``(H, W, 2)`` leído directamente
                del dispositivo V4L2 sin conversión RGB.

        Returns:
            Tuple de dos elementos:

            - ``heatmap`` (np.ndarray): Imagen BGR ``(H*scale, W*scale, 3)``
              con colormap y HUD superpuesto.
            - ``info`` (str): Métricas de temperatura en formato
              ``center_c=X.XX;avg_c=X.XX;min_c=X.XX;max_c=X.XX``.

        Raises:
            RuntimeError: Si el frame no tiene exactamente 2 canales o
                no es un array de 3 dimensiones.
        """
        if frame.ndim != 3 or frame.shape[2] != 2:
            raise RuntimeError(f'Formato de frame inesperado: {frame.shape}')

        half = frame.shape[0] // 2
        image_data   = frame[:half]
        thermal_data = frame[half:]

        center_temp  = self.pixel_to_celsius(thermal_data[96][128])
        min_position = np.unravel_index(np.argmin(thermal_data[..., 1]), thermal_data[..., 1].shape)
        max_position = np.unravel_index(np.argmax(thermal_data[..., 1]), thermal_data[..., 1].shape)
        min_temp     = self.pixel_to_celsius(thermal_data[min_position])
        max_temp     = self.pixel_to_celsius(thermal_data[max_position])
        avg_temp     = self.average_temperature(thermal_data)

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
        """Convierte un píxel de datos térmicos a temperatura en Celsius.

        La cámara codifica la temperatura en dos bytes por píxel:

        - ``pixel[0]``: byte alto (high byte).
        - ``pixel[1]``: byte bajo (low byte), multiplicado por 256.

        La fórmula aplica la escala de la cámara (÷64) y el offset
        de conversión Kelvin→Celsius (−273.15).

        Args:
            pixel: Array de 2 elementos ``[high_byte, low_byte]`` del canal
                de datos térmicos.

        Returns:
            Temperatura en grados Celsius como ``float``.
        """
        hi = float(pixel[0])
        lo = float(pixel[1]) * 256.0
        return ((hi + lo) / 64.0) - 273.15

    def average_temperature(self, thermal_data: np.ndarray) -> float:
        """Calcula la temperatura promedio de toda la escena térmica.

        Promedia independientemente los bytes altos y bajos del array de
        datos térmicos y aplica la misma fórmula de conversión que
        :meth:`pixel_to_celsius`.

        Args:
            thermal_data: Array NumPy de shape ``(H, W, 2)`` correspondiente
                a la mitad inferior del frame RAW (datos térmicos).

        Returns:
            Temperatura promedio de la escena en grados Celsius.
        """
        low_mean  = float(np.mean(thermal_data[..., 1])) * 256.0
        high_mean = float(np.mean(thermal_data[..., 0]))
        return ((high_mean + low_mean) / 64.0) - 273.15

    # ------------------------------------------------------------------
    # Dibujo HUD
    # ------------------------------------------------------------------

    def draw_crosshair(self, heatmap: np.ndarray, center_temp: float) -> None:
        """Dibuja una mira central con temperatura en el heatmap.

        Traza dos líneas perpendiculares de 40 px de largo centradas en la
        imagen con contorno negro (grosor 2) y relleno blanco (grosor 1) para
        legibilidad sobre cualquier colormap. Añade la etiqueta de temperatura
        con :meth:`put_label` en la esquina superior derecha de la mira.

        Args:
            heatmap: Frame BGR de destino (modificado in-place).
            center_temp: Temperatura del píxel central en grados Celsius.
        """
        height, width = heatmap.shape[:2]
        cx, cy = width // 2, height // 2
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
    ) -> None:
        """Dibuja un marcador circular en la posición de temperatura extrema.

        Escala las coordenadas del array térmico al espacio de la imagen
        redimensionada usando ``self.scale``. Dibuja un círculo relleno del
        color indicado con contorno negro y añade la etiqueta de temperatura.

        Convención de colores sugerida:
            - Rojo ``(0, 0, 255)`` → temperatura máxima.
            - Azul ``(255, 0, 0)`` → temperatura mínima.

        Args:
            heatmap: Frame BGR de destino (modificado in-place).
            pixel_position: Tupla ``(row, col)`` en coordenadas del array
                térmico (sin escalar).
            temperature: Temperatura en grados Celsius a mostrar en la etiqueta.
            color: Color BGR del círculo relleno.
        """
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
    ) -> None:
        """Dibuja el HUD de métricas de temperatura en la esquina superior izquierda.

        Renderiza un rectángulo negro semitransparente de 220×86 px y escribe
        5 líneas de texto en color cian con fuente ``FONT_HERSHEY_SIMPLEX``
        escala 0.4: promedio, centro, mínima, máxima y nombre del colormap.

        Args:
            heatmap: Frame BGR de destino (modificado in-place).
            avg_temp: Temperatura promedio de la escena en °C.
            center_temp: Temperatura del píxel central en °C.
            min_temp: Temperatura mínima detectada en °C.
            max_temp: Temperatura máxima detectada en °C.
        """
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
    def put_label(heatmap: np.ndarray, text: str, position: Tuple[int, int]) -> None:
        """Dibuja texto con contorno negro para legibilidad sobre cualquier fondo.

        Renderiza el texto dos veces: primero con grosor 2 en negro (contorno)
        y luego con grosor 1 en cian (relleno), logrando efecto de outline
        sin dependencias adicionales.

        Args:
            heatmap: Frame BGR de destino (modificado in-place).
            text: Cadena de texto a renderizar.
            position: Tupla ``(x, y)`` de la esquina inferior izquierda
                del texto en píxeles.
        """
        cv2.putText(heatmap, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0),    2, cv2.LINE_AA)
        cv2.putText(heatmap, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # Conversión a ROS Image
    # ------------------------------------------------------------------

    def cv_image_to_msg(self, image: np.ndarray) -> Image:
        """Convierte un array NumPy BGR a mensaje ``sensor_msgs/Image``.

        Rellena el header con timestamp actual y ``frame_id`` configurado.
        Garantiza que el array sea C-contiguo antes de serializar con
        ``tobytes()`` para evitar errores de memoria con vistas no contiguas.

        Args:
            image: Array NumPy BGR de shape ``(H, W, 3)``.

        Returns:
            Mensaje ``sensor_msgs/Image`` con encoding ``bgr8``, listo
            para publicar.
        """
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = image.shape[1] * image.shape[2]
        if not image.flags['C_CONTIGUOUS']:
            image = np.ascontiguousarray(image)
        msg.data = image.tobytes()
        return msg

    def destroy_node(self) -> None:
        """Libera el dispositivo de captura antes de destruir el nodo.

        Llama a ``capture.release()`` si el objeto existe, luego delega
        a ``super().destroy_node()`` para el ciclo de vida estándar de ROS2.
        """
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        super().destroy_node()


def main(args=None):
    """Punto de entrada del nodo de cámara térmica.

    Inicializa rclpy, instancia :class:`ThermalCameraNode` y lo mantiene
    activo con ``rclpy.spin``. Destruye el nodo limpiamente al recibir
    ``KeyboardInterrupt`` o al finalizar.

    Args:
        args: Argumentos de línea de comandos pasados a ``rclpy.init``.
            Defaults to None (usa ``sys.argv``).
    """
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