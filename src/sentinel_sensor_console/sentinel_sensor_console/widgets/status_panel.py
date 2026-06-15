from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGridLayout
from PySide6.QtCore import Qt

class StatusPanelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QWidget { background-color: #161B22; border: 1px solid #30363D; border-radius: 8px; }
            QLabel { border: none; }
        """)
        # Diccionario para guardar referencias a los labels de estado y de info
        self.status_labels = {} 
        self.info_labels = {}
        self._setup_ui()

    def _create_row(self, name, status, info, status_color="#3FB950"):
        row = QWidget()
        lyt = QHBoxLayout(row)
        lyt.setContentsMargins(0, 2, 0, 2)
        
        lbl_name = QLabel(name)
        lbl_name.setStyleSheet("color: #C9D1D9; font-size: 12px;")
        
        lbl_status = QLabel(status)
        lbl_status.setStyleSheet(f"color: {status_color}; font-size: 12px; font-weight: bold;")
        
        lbl_info = QLabel(info)
        lbl_info.setStyleSheet("color: #8B949E; font-size: 12px;")
        lbl_info.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        lyt.addWidget(lbl_name, stretch=3)
        lyt.addWidget(lbl_status, stretch=2)
        lyt.addWidget(lbl_info, stretch=3)
        
        return row, lbl_status, lbl_info

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        # Título
        title = QLabel("⚙ STATUS / DIAGNOSTICS")
        title.setStyleSheet("color: #8B949E; font-weight: bold; font-size: 12px;")
        layout.addWidget(title)

        # --- SECCIÓN SENSORES ---
        lbl_sensors = QLabel("SENSORS")
        lbl_sensors.setStyleSheet("color: #58A6FF; font-weight: bold; font-size: 11px; margin-top: 10px;")
        layout.addWidget(lbl_sensors)

        sensors = [
            ("Front Camera", "OK", "1920x1080 @ 30 FPS"),
            ("FishEye Camera", "OK", "1920x1080 @ 30 FPS"),
            ("RealSense", "OK", "1280x720 @ 30 FPS"),
            ("Thermal Camera", "OK", "640x480 @ 30 FPS"),
            ("Magnetometer", "OFFLINE", "I2C @ 100 Hz", "#F85149")
        ]
        for s in sensors:
            color = s[3] if len(s) > 3 else "#3FB950"
            row, _, _ = self._create_row(s[0], s[1], s[2], color)
            layout.addWidget(row)

        # --- SECCIÓN ALGORITMOS ---
        lbl_algos = QLabel("ALGORITHMS")
        lbl_algos.setStyleSheet("color: #58A6FF; font-weight: bold; font-size: 11px; margin-top: 15px;")
        layout.addWidget(lbl_algos)

        algos = [
            ("Landolt", "ACTIVE", "Orientation: -"),
            ("QR", "WAITING", "Text: -"),
            ("Motion", "STABLE", "Area: 0.0%"),
            ("YOLO 3D", "ACTIVE", "Objects: 0")
        ]
        
        for name, status, info in algos:
            row, lbl_status, lbl_info = self._create_row(name, status, info)
            self.status_labels[name] = lbl_status
            self.info_labels[name] = lbl_info
            layout.addWidget(row)

        layout.addStretch()

        # --- QUICK ACTIONS ---
        lbl_actions = QLabel("QUICK ACTIONS")
        lbl_actions.setStyleSheet("color: #8B949E; font-weight: bold; font-size: 12px; margin-bottom: 5px;")
        layout.addWidget(lbl_actions)

        actions_grid = QGridLayout()
        actions_grid.setSpacing(10)
        
        btn_style = """
            QPushButton { 
                background-color: #21262D; color: #C9D1D9; 
                border: 1px solid #363B42; border-radius: 4px; 
                padding: 10px; font-size: 12px; font-weight: bold; 
            }
            QPushButton:hover { background-color: #30363D; border: 1px solid #8B949E; }
        """
        btns = ["Start Sensors", "Restart Cameras", "Capture Landolt", "Save Snapshot", "Clear Captures", "Enable Debug"]
        for i, name in enumerate(btns):
            btn = QPushButton(name)
            btn.setStyleSheet(btn_style)
            actions_grid.addWidget(btn, i // 2, i % 2)
        
        layout.addLayout(actions_grid)

    def update_algo_status(self, name, status, info=None, color="#3FB950"):
        """Método maestro para actualizar cualquier fila de algoritmo"""
        if name in self.status_labels:
            self.status_labels[name].setText(status)
            self.status_labels[name].setStyleSheet(f"color: {color}; font-size: 12px; font-weight: bold;")
        if info is not None and name in self.info_labels:
            self.info_labels[name].setText(info)

    def update_qr_status(self, status):
        # Mantenemos compatibilidad con tu señal anterior
        color = "#3FB950" if status == "DETECTED" else "#D29922"
        self.update_algo_status("QR", status, color=color)