import math
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGridLayout, QFrame
from PySide6.QtCore import Qt, Signal, QPoint
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QPolygon, QFont

class CompassWidget(QWidget):
    """Brújula vectorial para el Magnetómetro"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 160)
        self.heading = 0.0

    def set_heading(self, angle):
        self.heading = angle
        self.update() # Fuerza a repintar el widget

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        size = min(rect.width(), rect.height()) - 10
        center = rect.center()

        # 1. Fondo de la brújula
        painter.setPen(QPen(QColor("#30363D"), 3))
        painter.setBrush(QColor("#0D1117"))
        painter.drawEllipse(center, size/2, size/2)

        # 2. Letras N, S, E, W
        painter.setPen(QColor("#8B949E"))
        font = QFont("Segoe UI", 9, QFont.Bold)
        painter.setFont(font)
        radius = size/2 - 15
        
        painter.drawText(center.x() - 10, int(center.y() - radius - 5), 20, 20, Qt.AlignCenter, "N")
        painter.drawText(center.x() - 10, int(center.y() + radius - 15), 20, 20, Qt.AlignCenter, "S")
        painter.drawText(int(center.x() + radius - 15), center.y() - 10, 20, 20, Qt.AlignCenter, "E")
        painter.drawText(int(center.x() - radius - 5), center.y() - 10, 20, 20, Qt.AlignCenter, "W")

        # 3. Dibujar la aguja rotada
        painter.translate(center)
        painter.rotate(self.heading)

        # Aguja Norte (Roja)
        north_poly = QPolygon([QPoint(-5, 0), QPoint(0, int(-size/2 + 20)), QPoint(5, 0)])
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#F85149"))
        painter.drawPolygon(north_poly)

        # Aguja Sur (Gris)
        south_poly = QPolygon([QPoint(-5, 0), QPoint(0, int(size/2 - 20)), QPoint(5, 0)])
        painter.setBrush(QColor("#8B949E"))
        painter.drawPolygon(south_poly)

        # Pin central
        painter.setBrush(QColor("#C9D1D9"))
        painter.drawEllipse(QPoint(-3, -3), 6, 6)


class StatusPanelWidget(QWidget):
    action_triggered = Signal(str) 

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QWidget { background-color: #161B22; border: 1px solid #30363D; border-radius: 8px; }
            QLabel { border: none; }
        """)
        self.status_labels = {} 
        self.info_labels = {}
        self._setup_ui()

    def _create_row(self, name, status, info, status_color):
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
        layout.setSpacing(10)

        title = QLabel("⚙ STATUS / DIAGNOSTICS")
        title.setStyleSheet("color: #8B949E; font-weight: bold; font-size: 12px;")
        layout.addWidget(title)

        # --- SENSORES ---
        lbl_sensors = QLabel("SENSORS")
        lbl_sensors.setStyleSheet("color: #58A6FF; font-weight: bold; font-size: 11px; margin-top: 5px;")
        layout.addWidget(lbl_sensors)

        sensors = [
            ("Front Camera", "OFFLINE", "Waiting...", "#F85149"),
            ("FishEye Camera", "OFFLINE", "Waiting...", "#F85149"),
            ("RealSense", "OFFLINE", "Waiting...", "#F85149"),
            ("Thermal Camera", "OFFLINE", "Waiting...", "#F85149"),
            ("Magnetometer", "OFFLINE", "I2C @ 100 Hz", "#F85149")
        ]
        for name, status, info, color in sensors:
            row, lbl_status, lbl_info = self._create_row(name, status, info, color)
            self.status_labels[name] = lbl_status
            self.info_labels[name] = lbl_info
            layout.addWidget(row)

        # --- ALGORITMOS ---
        lbl_algos = QLabel("ALGORITHMS")
        lbl_algos.setStyleSheet("color: #58A6FF; font-weight: bold; font-size: 11px; margin-top: 10px;")
        layout.addWidget(lbl_algos)

        algos = [
            ("Landolt", "WAITING", "Orientation: -", "#D29922"),
            ("QR", "WAITING", "Text: -", "#D29922"),
            ("Motion", "WAITING", "Area: 0.0%", "#D29922"),
            ("YOLO 3D", "WAITING", "Objects: 0", "#D29922")
        ]
        
        for name, status, info, color in algos:
            row, lbl_status, lbl_info = self._create_row(name, status, info, color)
            self.status_labels[name] = lbl_status
            self.info_labels[name] = lbl_info
            layout.addWidget(row)

        # 1. RESORTE SUPERIOR (Empuja la brújula hacia abajo)
        layout.addStretch(1)

        # --- MAGNÉTOMETRO VISUAL ---
        self.mag_container = QFrame()
        self.mag_container.setStyleSheet("background-color: #0D1117; border-radius: 6px; border: 1px solid #30363D;")
        
        # Cambiamos a QVBoxLayout para que sea de arriba hacia abajo
        mag_layout = QVBoxLayout(self.mag_container)
        mag_layout.setContentsMargins(10, 10, 10, 10)
        mag_layout.setSpacing(2) # Espacio pequeñito entre elementos
        
        mag_title = QLabel("Magnetometer")
        mag_title.setStyleSheet("color: #C9D1D9; font-weight: bold; font-size: 12px;")
        mag_title.setAlignment(Qt.AlignCenter) # Centramos el título
        
        self.lbl_heading = QLabel("0.0°")
        self.lbl_heading.setStyleSheet("color: #3FB950; font-weight: bold; font-size: 22px;")
        self.lbl_heading.setAlignment(Qt.AlignCenter) # Centramos los grados
        
        self.compass = CompassWidget()
        
        # Agregamos los elementos en orden vertical
        mag_layout.addWidget(mag_title)
        mag_layout.addWidget(self.lbl_heading)
        mag_layout.addWidget(self.compass, alignment=Qt.AlignCenter)
        
        layout.addWidget(self.mag_container)

        # 2. RESORTE INFERIOR (Empuja la brújula hacia arriba)
        layout.addStretch(1)

        # --- QUICK ACTIONS ---
        lbl_actions = QLabel("QUICK ACTIONS")
        lbl_actions.setStyleSheet("color: #8B949E; font-weight: bold; font-size: 14px; margin-bottom: 5px;")
        layout.addWidget(lbl_actions)

        # ¡AQUÍ ESTÁ LA LÍNEA QUE SE HABÍA PERDIDO!
        actions_grid = QGridLayout()
        actions_grid.setSpacing(10)
        
        btn_style = """
            QPushButton { 
                background-color: #21262D; color: #C9D1D9; 
                border: 1px solid #363B42; border-radius: 4px; 
                padding: 12px; font-size: 13px; font-weight: bold; 
            }
            QPushButton:hover { background-color: #30363D; border: 1px solid #8B949E; }
        """
        btns = ["Start Sensors", "Stop Sensors", "Reset Sensors", "Capture Landolt", "Save Snapshot", "Clear Captures"]
        for i, name in enumerate(btns):
            btn = QPushButton(name)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda checked=False, n=name: self.action_triggered.emit(n))
            actions_grid.addWidget(btn, i // 2, i % 2)
            
        layout.addLayout(actions_grid)

    def update_item_status(self, name, status, info=None, color="#3FB950"):
        if name in self.status_labels and self.status_labels[name].text() != status:
            self.status_labels[name].setText(status)
            self.status_labels[name].setStyleSheet(f"color: {color}; font-size: 12px; font-weight: bold;")
        if info is not None and name in self.info_labels and self.info_labels[name].text() != info:
            self.info_labels[name].setText(info)

    def update_magnetometer(self, heading):
        """Recibe los grados (0 a 360) y actualiza el UI"""
        self.lbl_heading.setText(f"{heading:.1f}°")
        self.compass.set_heading(heading)