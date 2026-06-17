import numpy as np
import cv2
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QGridLayout, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QImage, QPixmap


class CameraCell(QFrame):
    """Celda individual del grid de cámaras con selector de fuente y visor de video.

    Incluye un ``QComboBox`` para seleccionar el stream activo, una etiqueta
    de resolución/FPS, indicador de estado y el visor de video con cache de
    escala para evitar recálculos innecesarios.

    Args:
        cell_id: Identificador numérico usado en ``objectName``.
        parent: Widget padre (opcional).
    """

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

        # --- Barra superior ---
        header_layout = QHBoxLayout()

        self.combo = QComboBox()
        self.combo.setStyleSheet("""
            QComboBox {
                background-color: #0D1117; color: #C9D1D9;
                border: 1px solid #30363D; border-radius: 4px;
                padding: 3px 10px; font-size: 12px; font-weight: bold;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #161B22; color: white;
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

        # --- Visor de video ---
        self.video_label = QLabel("NO SIGNAL")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setStyleSheet(
            "background-color: #050608; border-radius: 4px; color: #484F58; border: none; font-weight: bold;"
        )

        layout.addLayout(header_layout)
        layout.addWidget(self.video_label, stretch=1)

        # Cache de escala: evita recalcular si no cambiaron el widget ni la fuente
        self._last_label_size = None
        self._last_src_size = None
        self._cached_scale_size = None

    def update_image(self, cv_image: np.ndarray) -> None:
        """Muestra un frame BGR en el visor de la celda.

        Convierte BGR→RGB con una view invertida (sin copia). Recalcula el
        tamaño de escala con ``SmoothTransformation`` solo cuando cambia el
        tamaño del widget o la resolución fuente; el render final usa
        ``FastTransformation`` sobre el tamaño ya cacheado.

        Args:
            cv_image: Frame BGR como array NumPy ``(H, W, 3)``. Si es
                ``None`` no hace nada.
        """
        if cv_image is None:
            return

        h, w = cv_image.shape[:2]

        rgb = cv_image[:, :, ::-1]
        if not rgb.flags['C_CONTIGUOUS']:
            rgb = np.ascontiguousarray(rgb)

        qimage = QImage(rgb.data, w, h, w * 3, QImage.Format_RGB888)

        label_size = self.video_label.size()
        if label_size != self._last_label_size or (w, h) != self._last_src_size:
            self._last_label_size = label_size
            self._last_src_size = (w, h)
            # Calcular tamaño sin crear pixmap
            src_ratio = w / h
            if label_size.height() == 0 or label_size.width() == 0:  # Evitar división por cero
                return
            lbl_ratio = label_size.width() / label_size.height()
            if src_ratio > lbl_ratio:
                sw = label_size.width()
                sh = int(sw / src_ratio)
            else:
                sh = label_size.height()
                sw = int(sh * src_ratio)
            self._cached_scale_size = QSize(sw, sh)

        pixmap = QPixmap.fromImage(qimage).scaled(
            self._cached_scale_size, Qt.IgnoreAspectRatio, Qt.FastTransformation
        )
        self.video_label.setPixmap(pixmap)


class CameraGridWidget(QWidget):
    """Grid 2×2 de celdas de cámara con enrutamiento por nombre de topic.

    Cada celda tiene un ``QComboBox`` con el nombre amigable del stream.
    ``update_image`` traduce el topic ROS al nombre amigable y entrega el
    frame a la celda que tenga ese nombre seleccionado.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        self.cells: list[CameraCell] = []
        for i in range(4):
            cell = CameraCell(i)
            self.cells.append(cell)
            row, col = divmod(i, 2)
            layout.addWidget(cell, row, col)

    def update_image(self, cv_image: np.ndarray, source_name: str) -> None:
        """Enruta un frame BGR a la celda que tenga el stream correspondiente activo.

        Traduce ``source_name`` (topic ROS) al nombre amigable del combo y
        busca la primera celda que lo tenga seleccionado. Si ninguna celda
        muestra ese stream, el frame se descarta silenciosamente.

        Args:
            cv_image: Frame BGR como array NumPy.
            source_name: Topic ROS de origen. Valores soportados:
                ``/usb_camera/image_raw``, ``/fisheye/image_raw``,
                ``/camera/color/image_raw``, ``/thermal_camera/image_raw``,
                ``/yolo/dbg_image``, ``/qr_detector/debug_image``,
                ``/image``, ``/motion_detector/debug_image``.
        """
        mapping = {
            "/usb_camera/image_raw":         "Front Camera",
            "/fisheye/image_raw":            "FishEye",
            "/camera/color/image_raw":       "RealSense",
            "/thermal_camera/image_raw":     "Thermal",
            "/yolo/dbg_image":               "YOLO 3D",
            "/qr_detector/debug_image":      "QR Capture",
            "/image":                        "Landolt Capture",
            "/motion_detector/debug_image":  "Motion Debug",
        }

        target_name = mapping.get(source_name)
        if target_name is None:
            return

        for cell in self.cells:
            if cell.combo.currentText() == target_name:
                cell.update_image(cv_image)