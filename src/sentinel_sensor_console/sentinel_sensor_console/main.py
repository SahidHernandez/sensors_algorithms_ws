import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PySide6.QtCore import Qt

from sentinel_sensor_console.widgets.camera_view import CameraGridWidget
from sentinel_sensor_console.widgets.algorithm_results import AlgorithmResultsWidget
from sentinel_sensor_console.widgets.status_panel import StatusPanelWidget
from sentinel_sensor_console.widgets.event_log import EventLogWidget
from sentinel_sensor_console.ros_qt_bridge import RosSignals, RosThread

class SentinelConsoleApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sentinel Sensor & Algorithm Console")
        self.resize(1380, 800)
        self.setStyleSheet("QMainWindow { background-color: #0D1117; } QLabel { color: white; font-family: 'Segoe UI', Arial, sans-serif; }")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # --- HEADER ---
        header_layout = QHBoxLayout()
        header_layout.addWidget(QLabel("⬢ SENTINEL SENSOR & ALGORITHM CONSOLE"))
        header_layout.addStretch()
        header_layout.addWidget(QLabel("   ✔ System: OK   "))
        main_layout.addLayout(header_layout)

        # --- CUERPO ---
        content_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        
        # Grid de cámaras
        self.camera_grid = CameraGridWidget()
        left_panel.addWidget(QLabel("CAMERA MONITOR"), stretch=0)
        left_panel.addWidget(self.camera_grid, stretch=6)
        
        # Algoritmos
        self.algo_results = AlgorithmResultsWidget(self)
        left_panel.addWidget(QLabel("ALGORITHM RESULTS"), stretch=0)
        left_panel.addWidget(self.algo_results, stretch=1) 
        
        # Logs
        self.event_log = EventLogWidget()
        left_panel.addWidget(self.event_log, stretch=2)
        
        content_layout.addLayout(left_panel, stretch=4) # Panel izquierdo ampliado
        
        # Panel Derecho
        self.status_panel = StatusPanelWidget()
        content_layout.addWidget(self.status_panel, stretch=1)
        
        main_layout.addLayout(content_layout)

        # --- CONEXIÓN DE SEÑALES ---
        self.ros_signals = RosSignals()
        
        # 1. Video (Cámaras Principales y Modales)
        self.ros_signals.new_image.connect(self.camera_grid.update_image)
        self.ros_signals.new_image.connect(self.algo_results.update_image)
        
        # 2. Algoritmo: QR
        self.ros_signals.qr_text_update.connect(self.algo_results.update_qr)
        self.ros_signals.qr_text_update.connect(lambda t: self.event_log.log_event("INFO", "QR", f"Detected: {t}"))
        self.ros_signals.qr_status_update.connect(self.status_panel.update_qr_status)
        self.ros_signals.qr_text_update.connect(lambda t: self.status_panel.update_algo_status("QR", "DETECTED", f"Text: {t}"))
        
        # 3. Algoritmo: Landolt
        self.ros_signals.landolt_orientation_update.connect(self.algo_results.update_landolt)
        self.ros_signals.landolt_orientation_update.connect(
            lambda orient: self.status_panel.update_algo_status("Landolt", "ACTIVE", f"Orientation: {orient}")
        )
        
        # 4. Algoritmo: Motion
        self.ros_signals.motion_status_update.connect(self.algo_results.update_motion)
        self.ros_signals.motion_status_update.connect(
            lambda stable, area: self.status_panel.update_algo_status(
                "Motion", 
                "STABLE" if stable else "MOTION", 
                f"Area: {area:.1f}%",
                "#3FB950" if stable else "#F85149" # Verde si estable, Rojo si movimiento
            )
        )
        
        # 5. Algoritmo: YOLO 3D
        self.ros_signals.yolo_text_update.connect(self.algo_results.update_yolo_text)
        self.ros_signals.yolo_text_update.connect(lambda t: self.event_log.log_event("INFO", "YOLO 3D", t.replace("\n", " ")))
        self.ros_signals.yolo_text_update.connect(
            lambda t: self.status_panel.update_algo_status("YOLO 3D", "ACTIVE", t.replace("\n", " "))
        )

        # --- ARRANCAR HILO ---
        self.ros_thread = RosThread(self.ros_signals)
        self.ros_thread.start()

    def closeEvent(self, event):
        print("Cerrando la consola y deteniendo el nodo de ROS 2 de forma segura...")
        self.ros_thread.stop()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = SentinelConsoleApp()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()