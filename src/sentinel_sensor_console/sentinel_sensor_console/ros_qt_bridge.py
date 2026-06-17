import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.context import Context

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32, Empty
from cv_bridge import CvBridge
from yolo_msgs.msg import DetectionArray

from PySide6.QtCore import QObject, QThread, Signal

import time 


class RosSignals(QObject):
    """Contenedor de señales Qt para comunicación thread-safe ROS2 → UI.

    Todas las señales se emiten desde ``SentinelConsoleNode`` (hilo ROS2)
    y se consumen en el hilo principal de PySide6.

    Signals:
        new_image (object, str): Frame BGR + nombre del topic de origen.
        qr_text_update (str): Texto decodificado del QR.
        qr_status_update (str): Estado del detector QR (e.g. ``"DETECTED"``).
        landolt_orientation_update (str): Orientación detectada por Landolt.
        motion_status_update (bool, float): Estabilidad y área de movimiento.
        yolo_text_update (str): Resumen de detecciones YOLO 3D.
        mag_heading_update (float): Heading del magnetómetro en grados.
    """

    new_image                  = Signal(object, str)
    qr_text_update             = Signal(str)
    qr_status_update           = Signal(str)
    landolt_orientation_update = Signal(str)
    motion_status_update       = Signal(bool, float)
    yolo_text_update           = Signal(str)
    mag_heading_update         = Signal(float)


class SentinelConsoleNode(Node):
    """Nodo ROS2 que suscribe a sensores y algoritmos y emite señales Qt.

    Actúa como bridge entre el ecosistema ROS2 y la UI PySide6. Las imágenes
    se throttlean a 15 FPS antes de emitir para no saturar el hilo de UI.
    Los datos de motion se agrupan: ``is_stable`` se almacena internamente y
    se emite junto con ``motion_area`` en cada callback de área.

    Publishers:
        /landolt/capture_command (Bool): Solicita captura manual a Landolt.
        /system/debug_mode (Bool): Activa/desactiva modo debug global.
        /system/restart_cameras (Empty): Reinicia el bringup de cámaras.

    Args:
        signals: Instancia de ``RosSignals`` donde se emiten las señales Qt.
        context: Contexto ROS2 dedicado (requerido para uso en hilo separado).
    """

    def __init__(self, signals: RosSignals, context=None):
        super().__init__('sentinel_console_node', context=context)
        self.signals = signals
        self.cv_bridge = CvBridge()

        self._last_emit_time: dict = {}
        self._emit_interval = 1.0 / 15.0  # Throttle a 15 FPS en la UI

        self.current_motion_stable = False

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Cámaras ---
        self.create_subscription(Image, '/usb_camera/image_raw',      self.usb_camera_callback, image_qos)
        self.create_subscription(Image, '/fisheye/image_raw',         self.fisheye_callback,    image_qos)
        self.create_subscription(Image, '/camera/color/image_raw',    self.realsense_callback,  image_qos)
        self.create_subscription(Image, '/thermal_camera/image_raw',  self.thermal_callback,    image_qos)

        # --- Algoritmos: video ---
        self.create_subscription(Image, '/qr_detector/debug_image',      self.qr_video_callback,     image_qos)
        self.create_subscription(Image, '/image',                         self.landolt_video_callback, image_qos)
        self.create_subscription(Image, '/motion_detector/debug_image',   self.motion_video_callback, image_qos)
        self.create_subscription(Image, '/yolo/dbg_image',                self.yolo_video_callback,   image_qos)

        # --- Algoritmos: datos ---
        self.create_subscription(String,         '/qr_detector/capture_text',  self.qr_text_callback,      reliable_qos)
        self.create_subscription(String,         '/captured/orientation',       self.landolt_orient_callback, reliable_qos)
        self.create_subscription(Bool,           '/motion_detector/is_stable',  self.motion_stable_callback, reliable_qos)
        self.create_subscription(Float32,        '/motion_detector/motion_area',self.motion_area_callback,  reliable_qos)
        self.create_subscription(DetectionArray, '/yolo/detections_3d',         self.yolo_data_callback,    reliable_qos)
        self.create_subscription(Float32,        '/magnetometer/heading',        self.mag_callback,          10)

        # --- Publishers ---
        self.pub_capture_landolt = self.create_publisher(Bool,  '/landolt/capture_command',  10)
        self.pub_enable_debug    = self.create_publisher(Bool,  '/system/debug_mode',        10)
        self.pub_restart_cams   = self.create_publisher(Empty, '/system/restart_cameras',   10)

        self.get_logger().info("ROS2 Bridge Node iniciado. Fase 1 y Fase 2 activas.")

    # ------------------------------------------------------------------
    # Callbacks de cámara
    # ------------------------------------------------------------------

    def usb_camera_callback(self, msg):   self._process_image(msg, "/usb_camera/image_raw")
    def fisheye_callback(self, msg):      self._process_image(msg, "/fisheye/image_raw")
    def realsense_callback(self, msg):    self._process_image(msg, "/camera/color/image_raw")
    def thermal_callback(self, msg):      self._process_image(msg, "/thermal_camera/image_raw")

    # ------------------------------------------------------------------
    # Callbacks de video de algoritmos
    # ------------------------------------------------------------------

    def qr_video_callback(self, msg):     self._process_image(msg, "/qr_detector/debug_image")
    def landolt_video_callback(self, msg):self._process_image(msg, "/image")
    def motion_video_callback(self, msg): self._process_image(msg, "/motion_detector/debug_image")
    def yolo_video_callback(self, msg):   self._process_image(msg, "/yolo/dbg_image")  # FIX: source_name corregido

    # ------------------------------------------------------------------
    # Procesamiento de imagen con throttle
    # ------------------------------------------------------------------

    def _process_image(self, msg, source_name: str) -> None:
        """Convierte un mensaje Image a BGR y lo emite con throttle a 15 FPS.

        Descarta el frame si no ha transcurrido ``_emit_interval`` desde la
        última emisión del mismo ``source_name``.

        Args:
            msg: Mensaje ``sensor_msgs/Image`` recibido.
            source_name: Topic de origen usado como clave de throttle e
                identificador en ``new_image``.
        """
        try:
            now = time.monotonic()
            if (now - self._last_emit_time.get(source_name, 0.0)) < self._emit_interval:
                return                                          # sale antes de convertir
            self._last_emit_time[source_name] = now
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.signals.new_image.emit(cv_image, source_name)
        except Exception as e:
            self.get_logger().error(f"Error imagen {source_name}: {e}")

    # ------------------------------------------------------------------
    # Callbacks de datos / texto
    # ------------------------------------------------------------------

    def qr_text_callback(self, msg: String) -> None:
        """Emite el texto QR y actualiza el status a DETECTED.

        Args:
            msg: Mensaje con el texto decodificado del QR.
        """
        # FIX: eliminado el .connect() que se registraba en cada mensaje
        self.signals.qr_text_update.emit(msg.data)
        self.signals.qr_status_update.emit("DETECTED")

    def landolt_orient_callback(self, msg: String) -> None:
        """Emite la orientación detectada por el algoritmo Landolt."""
        self.signals.landolt_orientation_update.emit(msg.data)

    def motion_stable_callback(self, msg: Bool) -> None:
        """Almacena el estado de estabilidad para combinarlo con el área."""
        self.current_motion_stable = msg.data

    def motion_area_callback(self, msg: Float32) -> None:
        """Emite el estado de motion agrupando estabilidad y área actual."""
        self.signals.motion_status_update.emit(self.current_motion_stable, msg.data)

    def mag_callback(self, msg: Float32) -> None:
        """Emite el heading del magnetómetro en grados."""
        self.signals.mag_heading_update.emit(msg.data)

    def yolo_data_callback(self, msg: DetectionArray) -> None:
        """Formatea las detecciones YOLO 3D y emite el resumen como texto.

        Agrupa los nombres de clase sin repetir usando un ``set``. Si no
        hay detecciones emite ``"No objects detected"``.

        Args:
            msg: Mensaje ``DetectionArray`` con lista de detecciones 3D.
        """
        num = len(msg.detections)
        if num == 0:
            self.signals.yolo_text_update.emit("No objects detected")
        else:
            detected_classes = {d.class_name for d in msg.detections}
            self.signals.yolo_text_update.emit(
                f"Objects: {num}\n[{', '.join(detected_classes)}]"
            )

    # ------------------------------------------------------------------
    # Comandos desde la UI
    # ------------------------------------------------------------------

    def execute_command(self, action_name: str) -> None:
        """Traduce una acción de la UI a un comando ROS2 publicado.

        Args:
            action_name: Nombre de la acción. Valores soportados:
                ``"Capture Landolt"``, ``"Enable Debug"``, ``"Restart Cameras"``,
                ``"Clear Captures"`` (sin efecto en ROS2, manejado en UI).
        """
        self.get_logger().info(f"UI command received: {action_name}")

        if action_name == "Capture Landolt":
            msg = Bool(); msg.data = True
            self.pub_capture_landolt.publish(msg)

        elif action_name == "Enable Debug":
            msg = Bool(); msg.data = True
            self.pub_enable_debug.publish(msg)

        elif action_name == "Restart Cameras":
            self.pub_restart_cams.publish(Empty())

        elif action_name == "Clear Captures":
            pass  # Manejado únicamente en la UI


class RosThread(QThread):
    """Hilo Qt que ejecuta el spin de ROS2 en un contexto dedicado.

    Usa un ``Context`` y ``SingleThreadedExecutor`` propios para aislarse
    del contexto global de rclpy y permitir shutdown limpio desde la UI.

    Args:
        signals: Instancia de ``RosSignals`` pasada a ``SentinelConsoleNode``.
    """

    def __init__(self, signals: RosSignals):
        super().__init__()
        self.signals  = signals
        self.node     = None
        self.executor = None
        self.context  = None

    def run(self) -> None:
        """Inicializa el contexto ROS2, crea el nodo y ejecuta el spin."""
        self.context = Context()
        rclpy.init(context=self.context)

        self.node = SentinelConsoleNode(self.signals, context=self.context)

        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        try:
            self.executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if rclpy.ok(context=self.context):
                self.executor.remove_node(self.node)
                self.node.destroy_node()
                rclpy.shutdown(context=self.context)

    def stop(self) -> None:
        """Solicita el shutdown del executor y espera a que el hilo termine."""
        if self.executor:
            self.executor.shutdown()
        self.wait()