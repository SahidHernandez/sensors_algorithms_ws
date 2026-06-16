import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
from yolo_msgs.msg import DetectionArray

from PySide6.QtCore import QObject, QThread, Signal

from std_msgs.msg import String, Bool, Float32, Empty

from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.context import Context

import rclpy.executors

class RosSignals(QObject):
    new_image = Signal(object, str)
    qr_text_update = Signal(str)
    landolt_orientation_update = Signal(str)
    motion_status_update = Signal(bool, float)
    yolo_text_update = Signal(str)
    qr_status_update = Signal(str)
    mag_heading_update = Signal(float)

class SentinelConsoleNode(Node):
    def __init__(self, signals: RosSignals, context=None):
        super().__init__('sentinel_console_node', context = context)
        self.signals = signals
        self.cv_bridge = CvBridge()

        self._last_emit_time: dict = {}
        self._emit_interval = 1.0 / 15.0  # throttle a 15fps en la UI
        
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- FASE 1: CÁMARAS ---
        self.create_subscription(Image, '/usb_camera/image_raw', self.usb_camera_callback, image_qos)
        self.create_subscription(Image, '/fisheye/image_raw', self.fisheye_callback, image_qos)
        self.create_subscription(Image, '/camera/color/image_raw', self.realsense_callback, image_qos)
        self.create_subscription(Image, '/thermal_camera/image_raw', self.thermal_callback, image_qos)
        
        # --- FASE 2: ALGORITMOS (IMÁGENES - VIDEO CONTINUO) ---
        self.create_subscription(Image, '/qr_detector/debug_image', self.qr_video_callback, image_qos)
        self.create_subscription(Image, '/image', self.landolt_video_callback, image_qos)
        self.create_subscription(Image, '/motion_detector/debug_image', self.motion_video_callback, image_qos)
        self.create_subscription(Image, '/yolo/dbg_image', self.yolo_video_callback, image_qos) # <-- Tópico corregido
        
        # --- FASE 2: ALGORITMOS (DATOS / TEXTO) ---
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, '/qr_detector/capture_text', self.qr_text_callback, reliable_qos)
        self.create_subscription(String, '/captured/orientation', self.landolt_orient_callback, reliable_qos)
        
        self.create_subscription(Bool, '/motion_detector/is_stable', self.motion_stable_callback, reliable_qos)
        self.create_subscription(Float32, '/motion_detector/motion_area', self.motion_area_callback, reliable_qos)
        self.create_subscription(DetectionArray, '/yolo/detections_3d', self.yolo_data_callback, reliable_qos) # <-- NUEVO

        self.sub_mag = self.create_subscription(
            Float32,
            '/magnetometer/heading', 
            self.mag_callback,
            10
        )

        self.pub_capture_landolt = self.create_publisher(Bool, '/landolt/capture_command', 10)
        self.pub_enable_debug = self.create_publisher(Bool, '/system/debug_mode', 10)
        self.pub_restart_cams = self.create_publisher(Empty, '/system/restart_cameras', 10)

        # Variables de estado interno para agrupar las señales de motion
        self.current_motion_stable = False

        self.get_logger().info("ROS 2 Bridge Node iniciado. Fase 1 y Fase 2 activas.")

    # Callbacks de Cámaras (usando el nombre del tópico como identificador)
    def usb_camera_callback(self, msg): self._process_image(msg, "/usb_camera/image_raw")
    def fisheye_callback(self, msg): self._process_image(msg, "/fisheye/image_raw")
    def realsense_callback(self, msg): self._process_image(msg, "/camera/color/image_raw")
    def thermal_callback(self, msg): self._process_image(msg, "/thermal_camera/image_raw")
    
    # Callbacks de Algoritmos
    def qr_video_callback(self, msg): self._process_image(msg, "/qr_detector/debug_image")
    def landolt_video_callback(self, msg): self._process_image(msg, "/image")
    def motion_video_callback(self, msg): self._process_image(msg, "/motion_detector/debug_image")
    def yolo_video_callback(self, msg): self._process_image(msg, "/yolo/dbg_image")

    def mag_callback(self, msg):
        # Transmite los grados a PySide6
        self.signals.mag_heading_update.emit(msg.data)

    def _process_image(self, msg, source_name):
        try:
            import time
            now = time.monotonic()
            last = self._last_emit_time.get(source_name, 0.0)
            if (now - last) < self._emit_interval:
                return
            self._last_emit_time[source_name] = now

            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.signals.new_image.emit(cv_image, source_name)
        except Exception as e:
            self.get_logger().error(f"Error imagen {source_name}: {e}")

    # --- Callbacks de Textos y Estados ---
    def qr_text_callback(self, msg):
        # 1. Actualiza la tarjeta de abajo (el texto grande)
        self.signals.qr_text_update.emit(msg.data)
        # 2. Actualiza el status panel lateral
        self.signals.qr_status_update.emit("DETECTED")
        # 3. Opcional: pasar el texto al status panel también
        self.signals.qr_text_update.connect(lambda t: self.status_panel.update_algo_status("QR", "DETECTED", f"Text: {t}"))

    def landolt_orient_callback(self, msg):
        self.signals.landolt_orientation_update.emit(msg.data)

    def motion_stable_callback(self, msg):
        self.current_motion_stable = msg.data

    def motion_area_callback(self, msg):
        self.signals.motion_status_update.emit(self.current_motion_stable, msg.data)

    def yolo_video_callback(self, msg): 
        self._process_image(msg, "YOLO 3D")

    def yolo_data_callback(self, msg):
        # msg.detections es una lista de objetos detectados en ese frame
        num_detections = len(msg.detections)
        
        if num_detections == 0:
            self.signals.yolo_text_update.emit("No objects detected")
        else:
            # Extraemos los nombres de las clases (ej: 'person', 'chair', 'bottle')
            # Usamos un Set para no repetir nombres (ej: en vez de "person, person", solo "person")
            detected_classes = set()
            for detection in msg.detections:
                detected_classes.add(detection.class_name)
            
            # Formateamos el texto final
            class_names_str = ", ".join(detected_classes)
            status_text = f"Objects: {num_detections}\n[{class_names_str}]"
            
            self.signals.yolo_text_update.emit(status_text)

    def execute_command(self, action_name):
        """Recibe el string de la interfaz y lo traduce a comandos de ROS 2"""
        self.get_logger().info(f"UI command received: {action_name}")
        
        if action_name == "Capture Landolt":
            msg = Bool()
            msg.data = True
            self.pub_capture_landolt.publish(msg)
            
        elif action_name == "Enable Debug":
            msg = Bool()
            msg.data = True # O haz un toggle (True/False)
            self.pub_enable_debug.publish(msg)
            
        elif action_name == "Restart Cameras":
            msg = Empty()
            self.pub_restart_cams.publish(msg)
            
        elif action_name == "Clear Captures":
            # Si esto solo limpia la UI, puedes emitir una señal de vuelta al main.
            # Si limpia un buffer en ROS, publica a un tópico.
            pass


class RosThread(QThread):
    def __init__(self, signals):
        super().__init__()
        self.signals = signals
        self.node = None
        self.executor = None
        self.context = None

    def run(self):
        # 1. Create a dedicated context for this thread
        self.context = Context()
        rclpy.init(context=self.context)

        # 2. Instantiate the node within this context
        self.node = SentinelConsoleNode(self.signals, context=self.context)
        
        # 3. Use a dedicated executor bound to the context
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

    def stop(self):
        if self.executor:
            self.executor.shutdown()
        self.wait()