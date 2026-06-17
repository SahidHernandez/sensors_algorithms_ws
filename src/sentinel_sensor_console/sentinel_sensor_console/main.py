import sys
import os
import signal
import subprocess
from datetime import datetime
from PySide6.QtCore import Qt, QTimer

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PySide6.QtCore import Qt

from sentinel_sensor_console.widgets.camera_view import CameraGridWidget
from sentinel_sensor_console.widgets.algorithm_results import AlgorithmResultsWidget
from sentinel_sensor_console.widgets.status_panel import StatusPanelWidget
from sentinel_sensor_console.widgets.event_log import EventLogWidget
from sentinel_sensor_console.ros_qt_bridge import RosSignals, RosThread


class SentinelConsoleApp(QMainWindow):
    """Ventana principal de la consola Sentinel.

    Integra el grid de cámaras, resultados de algoritmos, panel de estado
    y log de eventos. Gestiona el hilo ROS2, el proceso de launch de sensores
    y las acciones rápidas del operador.

    Attributes:
        camera_grid (CameraGridWidget): Grid 2×2 de streams de cámara.
        algo_results (AlgorithmResultsWidget): Tarjetas de resultado por algoritmo.
        event_log (EventLogWidget): Tabla de eventos del sistema.
        status_panel (StatusPanelWidget): Panel de diagnóstico y acciones rápidas.
        ros_signals (RosSignals): Señales Qt emitidas desde el hilo ROS2.
        ros_thread (RosThread): Hilo dedicado al spin de ROS2.
        active_streams (set[str]): Topics que ya han enviado al menos un frame.
        launch_process (subprocess.Popen | None): Proceso del launch de sensores activo.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sentinel Sensor & Algorithm Console")
        self.resize(1380, 800)
        self.setStyleSheet(
            "QMainWindow { background-color: #0D1117; } "
            "QLabel { color: white; font-family: 'Segoe UI', Arial, sans-serif; }"
        )

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # --- Header ---
        header_layout = QHBoxLayout()
        header_layout.addWidget(QLabel("⬢ SENTINEL SENSOR & ALGORITHM CONSOLE"))
        header_layout.addStretch()
        header_layout.addWidget(QLabel("   ✔ System: OK   "))
        main_layout.addLayout(header_layout)

        # --- Cuerpo ---
        content_layout = QHBoxLayout()
        left_panel = QVBoxLayout()

        self.camera_grid = CameraGridWidget()
        left_panel.addWidget(QLabel("CAMERA MONITOR"), stretch=0)
        left_panel.addWidget(self.camera_grid, stretch=10)

        self.algo_results = AlgorithmResultsWidget(self)
        left_panel.addWidget(QLabel("ALGORITHM RESULTS"), stretch=0)
        left_panel.addWidget(self.algo_results, stretch=0)

        self.event_log = EventLogWidget()
        left_panel.addWidget(self.event_log, stretch=3)

        content_layout.addLayout(left_panel, stretch=5)

        self.status_panel = StatusPanelWidget()
        content_layout.addWidget(self.status_panel, stretch=1)

        main_layout.addLayout(content_layout)

        self.active_streams: set = set()
        self.launch_process: subprocess.Popen | None = None

        # --- Señales ROS2 ---
        self.ros_signals = RosSignals()

        # Video → grid, modales y detección de streams activos
        self.ros_signals.new_image.connect(self.camera_grid.update_image)
        self.ros_signals.new_image.connect(self.algo_results.update_image)
        self.ros_signals.new_image.connect(self._check_stream_active)

        # QR
        self.ros_signals.qr_text_update.connect(self.algo_results.update_qr)
        self.ros_signals.qr_text_update.connect(
            lambda t: self.event_log.log_event("INFO", "QR", f"Detected: {t}")
        )
        self.ros_signals.qr_status_update.connect(
            lambda s: self.status_panel.update_item_status(
                "QR", s, color="#3FB950" if s == "DETECTED" else "#D29922"
            )
        )
        self.ros_signals.qr_text_update.connect(
            lambda t: self.status_panel.update_item_status("QR", "DETECTED", f"Text: {t}")
        )

        # Landolt
        self.ros_signals.landolt_orientation_update.connect(self.algo_results.update_landolt)
        self.ros_signals.landolt_orientation_update.connect(
            lambda orient: self.status_panel.update_item_status("Landolt", "ACTIVE", f"Orientation: {orient}")
        )

        # Motion
        self.ros_signals.motion_status_update.connect(self.algo_results.update_motion)
        self.ros_signals.motion_status_update.connect(
            lambda stable, area: self.status_panel.update_item_status(
                "Motion",
                "STABLE" if stable else "MOTION",
                f"Area: {area:.1f}%",
                "#3FB950" if stable else "#F85149"
            )
        )

        # YOLO 3D
        self.ros_signals.yolo_text_update.connect(self.algo_results.update_yolo_text)
        self.ros_signals.yolo_text_update.connect(
            lambda t: self.event_log.log_event("INFO", "YOLO 3D", t.replace("\n", " "))
        )
        self.ros_signals.yolo_text_update.connect(
            lambda t: self.status_panel.update_item_status("YOLO 3D", "ACTIVE", t.replace("\n", " "))
        )

        # Magnetómetro
        self.ros_signals.mag_heading_update.connect(self.status_panel.update_magnetometer)
        self.ros_signals.mag_heading_update.connect(
            lambda: self.status_panel.update_item_status("Magnetometer", "OK", "I2C @ 100 Hz", "#3FB950")
        )

        self.status_panel.action_triggered.connect(self._handle_quick_action)

        # --- Hilo ROS2 ---
        self.ros_thread = RosThread(self.ros_signals)
        self.ros_thread.start()

    def _check_stream_active(self, cv_image, source_name):
        """Marca un stream como activo la primera vez que recibe un frame.

        Traduce ``source_name`` al nombre del sensor en el panel de estado
        y actualiza su status a ``"OK"`` o ``"ACTIVE"`` con la resolución
        del frame. Las llamadas posteriores del mismo topic se ignoran.

        Args:
            cv_image: Frame recibido (solo se usa para leer resolución).
            source_name: Topic ROS o nombre amigable del stream.
        """
        if source_name not in self.active_streams:
            self.active_streams.add(source_name)

            target_name = None
            if   "usb_camera"    in source_name or source_name == "Front Camera":   target_name = "Front Camera"
            elif "fisheye"       in source_name or source_name == "FishEye Camera": target_name = "FishEye Camera"
            elif "camera/color"  in source_name or source_name == "RealSense":      target_name = "RealSense"
            elif "thermal"       in source_name or source_name == "Thermal Camera": target_name = "Thermal Camera"
            elif "yolo"          in source_name or source_name == "YOLO 3D":        target_name = "YOLO 3D"
            elif "qr_detector"   in source_name or source_name == "QR Video":       target_name = "QR"
            elif "/image"        == source_name  or source_name == "Landolt Video": target_name = "Landolt"
            elif "motion"        in source_name  or source_name == "Motion Video":  target_name = "Motion"

            if target_name:
                h, w = cv_image.shape[:2]
                is_sensor = target_name in ["Front Camera", "FishEye Camera", "RealSense", "Thermal Camera"]
                self.status_panel.update_item_status(
                    target_name,
                    "OK" if is_sensor else "ACTIVE",
                    f"{w}x{h} Stream",
                    "#3FB950"
                )
            
            # Si ya están todos los streams activos, desconectar el slot
            expected = {
                "/usb_camera/image_raw",
                "/fisheye/image_raw",
                "/camera/color/image_raw",
                "/thermal_camera/image_raw",
                "/thermal_camera/image_raw",
                "/qr_detector/debug_image",
                "/motion_detector/debug_image",
                "/yolo/dbg_image",
                "/image",
            }
            if expected.issubset(self.active_streams):
                try:
                    self.ros_signals.new_image.disconnect(self._check_stream_active)
                except RuntimeError:
                    pass  # Ya estaba desconectado

    def _handle_quick_action(self, action_name: str) -> None:
        """Ejecuta la acción rápida seleccionada en el panel de estado.

        Acciones disponibles:

        - ``Start Sensors``: Lanza ``full_sensors_algorithms.launch.py`` en un
          subproceso con ``os.setsid`` para gestión de grupo de procesos.
        - ``Stop Sensors``: Envía ``SIGINT`` al grupo del proceso de launch y
          resetea el estado de la UI sin bloquear el hilo principal.
        - ``Reset Sensors``: Detiene y relanza con 3 s de espera vía ``QTimer``.
        - ``Save Snapshot``: Captura la ventana con ``QMainWindow.grab()`` y
          guarda un PNG con timestamp.
        - ``Clear Captures``: Resetea los labels de QR, YOLO y Landolt.
        - ``Capture Landolt``: Loggea la solicitud de captura manual.

        Args:
            action_name: Nombre del botón pulsado en ``StatusPanelWidget``.
        """
        self.event_log.log_event("WARN", "SYSTEM", f"Executing: {action_name}")

        if action_name == "Start Sensors":
            if self.launch_process is None or self.launch_process.poll() is not None:
                self.event_log.log_event("INFO", "SYSTEM", "Launching sensors and algorithms...")
                self.launch_process = subprocess.Popen(
                    ['ros2', 'launch', 'camera_bringup', 'full_sensors_algorithms.launch.py'],
                    preexec_fn=os.setsid
                )
            else:
                self.event_log.log_event("WARN", "SYSTEM", "Sensors are already running.")

        elif action_name == "Stop Sensors":
            if self.launch_process is not None and self.launch_process.poll() is None:
                self.event_log.log_event("ERROR", "SYSTEM", "Sending shutdown signal to all nodes...")
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGINT)
                self.launch_process = None
                self._reset_ui_status()
                self.event_log.log_event("INFO", "SYSTEM", "Shutdown signal sent. UI reset.")
            else:
                self.event_log.log_event("WARN", "SYSTEM", "No active sensors to stop.")

        elif action_name == "Reset Sensors":
            self.event_log.log_event("WARN", "SYSTEM", "Initiating full reset...")
            self._handle_quick_action("Stop Sensors")
            QTimer.singleShot(3000, lambda: self._handle_quick_action("Start Sensors"))

        elif action_name == "Save Snapshot":
            pixmap = self.grab()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"sentinel_snapshot_{timestamp}.png"
            pixmap.save(filename)
            self.event_log.log_event("INFO", "SYSTEM", f"Snapshot saved: {filename}")

        elif action_name == "Clear Captures":
            self.algo_results.update_qr("Waiting...")
            self.algo_results.update_yolo_text("No objects detected")
            self.algo_results.update_landolt("Waiting...")
            self.event_log.log_event("INFO", "SYSTEM", "UI captures cleared.")

        elif action_name == "Capture Landolt":
            self.event_log.log_event("INFO", "LANDOLT", "Manual capture requested.")

    def _reset_ui_status(self) -> None:
        """Devuelve sensores y algoritmos a su estado inicial offline/waiting.

        Limpia ``active_streams`` y actualiza el panel de estado:
        cámaras → ``OFFLINE`` (rojo), algoritmos → ``WAITING`` (ámbar).
        """
        self.active_streams.clear()
        for cam in ["Front Camera", "FishEye Camera", "RealSense", "Thermal Camera"]:
            self.status_panel.update_item_status(cam, "OFFLINE", "Waiting...", "#F85149")
        for algo in ["Landolt", "QR", "Motion", "YOLO 3D"]:
            self.status_panel.update_item_status(algo, "WAITING", "-", "#D29922")

    def closeEvent(self, event) -> None:
        """Detiene el proceso de launch y el hilo ROS2 al cerrar la ventana.

        Envía ``SIGINT`` al grupo de procesos del launch sin ``wait()``
        para no bloquear el cierre de la UI.
        """
        print("Cerrando la consola y deteniendo nodos...")
        if hasattr(self, 'launch_process') and self.launch_process is not None and self.launch_process.poll() is None:
            os.killpg(os.getpgid(self.launch_process.pid), signal.SIGINT)
        self.ros_thread.stop()
        event.accept()


def main(args=None):
    """Punto de entrada de la aplicación Sentinel Console."""
    app = QApplication(sys.argv)
    window = SentinelConsoleApp()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()