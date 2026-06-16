import math
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGridLayout, QFrame
from PySide6.QtCore import Qt, Signal, QPoint
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QPolygon, QFont


class CompassWidget(QWidget):
    """Brújula vectorial dibujada con QPainter.

    Renderiza un círculo con puntos cardinales (N/S/E/W) y una aguja
    bicolor rotada según el heading: punta norte en rojo, sur en gris.

    Args:
        parent: Widget padre (opcional).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 160)
        self.heading = 0.0

    def set_heading(self, angle: float) -> None:
        """Actualiza el ángulo de la aguja y fuerza un repintado.

        Args:
            angle: Heading en grados (0–360, sentido horario desde Norte).
        """
        self.heading = angle
        self.update()

    def paintEvent(self, event) -> None:
        """Dibuja el fondo, cardinales y aguja rotada según ``self.heading``."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        size = min(rect.width(), rect.height()) - 10
        center = rect.center()

        # Fondo circular
        painter.setPen(QPen(QColor("#30363D"), 3))
        painter.setBrush(QColor("#0D1117"))
        painter.drawEllipse(center, size/2, size/2)

        # Puntos cardinales
        painter.setPen(QColor("#8B949E"))
        font = QFont("Segoe UI", 9, QFont.Bold)
        painter.setFont(font)
        radius = size/2 - 15

        painter.drawText(center.x() - 10, int(center.y() - radius - 5),  20, 20, Qt.AlignCenter, "N")
        painter.drawText(center.x() - 10, int(center.y() + radius - 15), 20, 20, Qt.AlignCenter, "S")
        painter.drawText(int(center.x() + radius - 15), center.y() - 10, 20, 20, Qt.AlignCenter, "E")
        painter.drawText(int(center.x() - radius - 5),  center.y() - 10, 20, 20, Qt.AlignCenter, "W")

        # Aguja rotada al heading
        painter.translate(center)
        painter.rotate(self.heading)

        # Punta Norte (roja)
        north_poly = QPolygon([QPoint(-5, 0), QPoint(0, int(-size/2 + 20)), QPoint(5, 0)])
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#F85149"))
        painter.drawPolygon(north_poly)

        # Punta Sur (gris)
        south_poly = QPolygon([QPoint(-5, 0), QPoint(0, int(size/2 - 20)), QPoint(5, 0)])
        painter.setBrush(QColor("#8B949E"))
        painter.drawPolygon(south_poly)

        # Pin central
        painter.setBrush(QColor("#C9D1D9"))
        painter.drawEllipse(QPoint(-3, -3), 6, 6)


class StatusPanelWidget(QWidget):
    """Panel lateral de estado, diagnóstico y acciones rápidas.

    Muestra el estado de sensores y algoritmos en filas nombre/status/info,
    una brújula visual del magnetómetro y un grid de botones de acción rápida.

    Signals:
        action_triggered (str): Emitido al pulsar un botón de acción.
            El valor es el nombre del botón (e.g. ``"Start Sensors"``).

    Attributes:
        status_labels (dict[str, QLabel]): Labels de status indexados por nombre.
        info_labels (dict[str, QLabel]): Labels de info indexados por nombre.
    """

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

    def _create_row(self, name: str, status: str, info: str, status_color: str):
        """Crea una fila horizontal con nombre, status e info.

        Args:
            name: Nombre del sensor o algoritmo (columna izquierda).
            status: Texto de estado inicial (columna central, en negrita).
            info: Texto informativo inicial (columna derecha, alineado a la derecha).
            status_color: Color CSS del label de status (e.g. ``"#F85149"``).

        Returns:
            Tuple ``(row, lbl_status, lbl_info)`` donde ``row`` es el
            ``QWidget`` contenedor listo para añadir al layout.
        """
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

        lyt.addWidget(lbl_name,   stretch=3)
        lyt.addWidget(lbl_status, stretch=2)
        lyt.addWidget(lbl_info,   stretch=3)

        return row, lbl_status, lbl_info

    def _setup_ui(self):
        """Construye el layout completo: sensores, algoritmos, brújula y acciones."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(10)

        title = QLabel("⚙ STATUS / DIAGNOSTICS")
        title.setStyleSheet("color: #8B949E; font-weight: bold; font-size: 12px;")
        layout.addWidget(title)

        # --- Sensores ---
        lbl_sensors = QLabel("SENSORS")
        lbl_sensors.setStyleSheet("color: #58A6FF; font-weight: bold; font-size: 11px; margin-top: 5px;")
        layout.addWidget(lbl_sensors)

        sensors = [
            ("Front Camera",   "OFFLINE", "Waiting...",  "#F85149"),
            ("FishEye Camera", "OFFLINE", "Waiting...",  "#F85149"),
            ("RealSense",      "OFFLINE", "Waiting...",  "#F85149"),
            ("Thermal Camera", "OFFLINE", "Waiting...",  "#F85149"),
            ("Magnetometer",   "OFFLINE", "I2C @ 100 Hz","#F85149"),
        ]
        for name, status, info, color in sensors:
            row, lbl_status, lbl_info = self._create_row(name, status, info, color)
            self.status_labels[name] = lbl_status
            self.info_labels[name]   = lbl_info
            layout.addWidget(row)

        # --- Algoritmos ---
        lbl_algos = QLabel("ALGORITHMS")
        lbl_algos.setStyleSheet("color: #58A6FF; font-weight: bold; font-size: 11px; margin-top: 10px;")
        layout.addWidget(lbl_algos)

        algos = [
            ("Landolt", "WAITING", "Orientation: -", "#D29922"),
            ("QR",      "WAITING", "Text: -",        "#D29922"),
            ("Motion",  "WAITING", "Area: 0.0%",     "#D29922"),
            ("YOLO 3D", "WAITING", "Objects: 0",     "#D29922"),
        ]
        for name, status, info, color in algos:
            row, lbl_status, lbl_info = self._create_row(name, status, info, color)
            self.status_labels[name] = lbl_status
            self.info_labels[name]   = lbl_info
            layout.addWidget(row)

        layout.addStretch(1)

        # --- Magnetómetro visual ---
        self.mag_container = QFrame()
        self.mag_container.setStyleSheet(
            "background-color: #0D1117; border-radius: 6px; border: 1px solid #30363D;"
        )

        mag_layout = QVBoxLayout(self.mag_container)
        mag_layout.setContentsMargins(10, 10, 10, 10)
        mag_layout.setSpacing(2)

        mag_title = QLabel("Magnetometer")
        mag_title.setStyleSheet("color: #C9D1D9; font-weight: bold; font-size: 12px;")
        mag_title.setAlignment(Qt.AlignCenter)

        self.lbl_heading = QLabel("0.0°")
        self.lbl_heading.setStyleSheet("color: #3FB950; font-weight: bold; font-size: 22px;")
        self.lbl_heading.setAlignment(Qt.AlignCenter)

        self.compass = CompassWidget()

        mag_layout.addWidget(mag_title)
        mag_layout.addWidget(self.lbl_heading)
        mag_layout.addWidget(self.compass, alignment=Qt.AlignCenter)

        layout.addWidget(self.mag_container)

        layout.addStretch(1)

        # --- Quick Actions ---
        lbl_actions = QLabel("QUICK ACTIONS")
        lbl_actions.setStyleSheet("color: #8B949E; font-weight: bold; font-size: 14px; margin-bottom: 5px;")
        layout.addWidget(lbl_actions)

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

    def update_item_status(self, name: str, status: str, info: str = None, color: str = "#3FB950") -> None:
        """Actualiza el status e info de un sensor o algoritmo.

        Solo actualiza el label si el texto cambió, para evitar repaints innecesarios.

        Args:
            name: Clave del item (debe existir en ``status_labels``).
            status: Nuevo texto de estado.
            info: Nuevo texto informativo. ``None`` para no modificarlo.
            color: Color CSS del texto de status. Defaults to ``"#3FB950"`` (verde).
        """
        if name in self.status_labels and self.status_labels[name].text() != status:
            self.status_labels[name].setText(status)
            self.status_labels[name].setStyleSheet(f"color: {color}; font-size: 12px; font-weight: bold;")
        if info is not None and name in self.info_labels and self.info_labels[name].text() != info:
            self.info_labels[name].setText(info)

    def update_magnetometer(self, heading: float) -> None:
        """Actualiza el label de grados y la brújula visual.

        Args:
            heading: Heading en grados (0–360).
        """
        self.lbl_heading.setText(f"{heading:.1f}°")
        self.compass.set_heading(heading)