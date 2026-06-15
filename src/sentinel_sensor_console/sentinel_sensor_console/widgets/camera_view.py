import numpy as np
import cv2
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QGridLayout, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap


class CameraCell(QFrame):
    """Una sola celda de cámara con su menú, info de FPS y video."""
    def __init__(self, cell_id, parent=None):
        super().__init__(parent)
        self.setObjectName(f"CameraCell_{cell_id}")

        self.setStyleSheet("""
            QFrame {
                background-color: #161B22;
                border: 1px solid #30363D;
                border-radius: 8px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(8)

        # --- Barra Superior de la Celda ---
        header_layout = QHBoxLayout()

        self.combo = QComboBox()
        self.combo.setStyleSheet("""
            QComboBox {
                background-color: #0D1117;
                color: #C9D1D9;
                border: 1px solid #30363D;
                border-radius: 4px;
                padding: 3px 10px;
                font-size: 12px;
                font-weight: bold;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #161B22;
                color: white;
                selection-background-color: #238636;
            }
        """)

        self.topic_options = [
            "Front Camera", "FishEye", "RealSense", "Thermal",
            "Motion Debug", "QR Capture", "Landolt Capture", "YOLO 3D", "None"
        ]
        self.combo.addItems(self.topic_options)

        self.info_label = QLabel("1920x1080   30 FPS")
        self.info_label.setStyleSheet("color: #8B949E; border: none; font-size: 11px;")

        self.status_dot = QLabel("●")
        self.status_dot.setStyleSheet("color: #238636; border: none; font-size: 14px;")

        header_layout.addWidget(self.combo)
        header_layout.addStretch()
        header_layout.addWidget(self.info_label)
        header_layout.addWidget(self.status_dot)

        # --- Visor de Video ---
        self.video_label = QLabel("NO SIGNAL")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setStyleSheet(
            "background-color: #050608; border-radius: 4px; color: #484F58; border: none; font-weight: bold;"
        )

        layout.addLayout(header_layout)
        layout.addWidget(self.video_label, stretch=1)

        # Cache para scaling
        self._last_label_size = None
        self._last_src_size = None
        self._cached_scale_size = None

    def update_image(self, cv_image):
        if cv_image is None:
            return

        h, w = cv_image.shape[:2]

        # BGR→RGB sin copia extra (view invertida)
        rgb = cv_image[:, :, ::-1]
        if not rgb.flags['C_CONTIGUOUS']:
            rgb = np.ascontiguousarray(rgb)

        qimage = QImage(rgb.data, w, h, w * 3, QImage.Format_RGB888)

        # Recalcula el tamaño de escala solo si cambió el widget o la fuente
        label_size = self.video_label.size()
        if label_size != self._last_label_size or (w, h) != self._last_src_size:
            self._last_label_size = label_size
            self._last_src_size = (w, h)
            self._cached_scale_size = QPixmap.fromImage(qimage).scaled(
                label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
            ).size()

        pixmap = QPixmap.fromImage(qimage).scaled(
            self._cached_scale_size, Qt.IgnoreAspectRatio, Qt.FastTransformation
        )
        self.video_label.setPixmap(pixmap)


class CameraGridWidget(QWidget):
    """La cuadrícula 2x2 que contiene las 4 cámaras."""
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        self.cells = []
        for i in range(4):
            cell = CameraCell(i)
            self.cells.append(cell)
            row, col = divmod(i, 2)
            layout.addWidget(cell, row, col)

    def update_image(self, cv_image, source_name):
        """Recibe la imagen y el nombre del tópico y la enruta a la celda correcta."""
        mapping = {
            "/usb_camera/image_raw":          "Front Camera",
            "/fisheye/image_raw":             "FishEye",
            "/camera/color/image_raw":        "RealSense",
            "/thermal_camera/image_raw":      "Thermal",
            "/yolo/dbg_image":                "YOLO 3D",
            "/qr_detector/debug_image":       "QR Capture",
            "/image":                         "Landolt Capture",
            "/motion_detector/debug_image":   "Motion Debug",
        }

        target_name = mapping.get(source_name)
        if target_name is None:
            return

        for cell in self.cells:
            if cell.combo.currentText() == target_name:
                cell.update_image(cv_image)