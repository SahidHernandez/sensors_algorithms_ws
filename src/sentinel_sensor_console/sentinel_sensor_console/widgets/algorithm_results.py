import cv2
import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDialog, QFrame
from PySide6.QtCore import Qt, QEvent
from PySide6.QtGui import QImage, QPixmap


class VideoModalDialog(QDialog):
    """Modal fullscreen para mostrar un stream de video continuo.

    Se ancla y redimensiona sobre el widget padre automáticamente.
    El fondo es semitransparente (rgba 10,12,16 / 220).

    Args:
        title: Texto del encabezado del modal.
        parent: Widget padre (opcional).
    """

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setModal(True)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Tool)
        self.setAttribute(Qt.WA_TranslucentBackground)

        if self.parentWidget():
            self.parentWidget().installEventFilter(self)

        self.bg_frame = QFrame(self)
        self.bg_frame.setStyleSheet("background-color: rgba(10, 12, 16, 220);")

        layout = QVBoxLayout(self.bg_frame)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(15)

        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")
        self.title_label.setAlignment(Qt.AlignCenter)

        self.video_label = QLabel("Waiting for video stream...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setFixedSize(800, 600)
        self.video_label.setStyleSheet(
            "background-color: black; border: 2px solid rgb(80, 90, 105); color: white; font-size: 16px;"
        )

        self.close_btn = QPushButton("Close Video")
        self.close_btn.setFixedSize(150, 40)
        self.close_btn.setStyleSheet("""
            QPushButton { background-color: rgb(190, 30, 30); color: white; font-weight: bold; border-radius: 6px; font-size: 14px;}
            QPushButton:hover { background-color: rgb(230, 40, 40); }
        """)
        self.close_btn.clicked.connect(self.hide)

        layout.addWidget(self.title_label, alignment=Qt.AlignCenter)
        layout.addWidget(self.video_label, alignment=Qt.AlignCenter)
        layout.addWidget(self.close_btn, alignment=Qt.AlignCenter)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(self.bg_frame)

        self.cached_size = self.video_label.size()
        self.cached_pixmap = QPixmap()

    def eventFilter(self, obj, event):
        """Mantiene el modal alineado al padre en resize y move."""
        if obj == self.parentWidget() and event.type() in (QEvent.Resize, QEvent.Move):
            self.setGeometry(self.parentWidget().geometry())
        return super().eventFilter(obj, event)

    def showEvent(self, event):
        """Ajusta geometría al padre al mostrarse."""
        if self.parentWidget():
            self.setGeometry(self.parentWidget().geometry())
        super().showEvent(event)

    def update_image(self, cv_image: np.ndarray) -> None:
        """Convierte un frame BGR a QPixmap y lo muestra escalado.

        No hace nada si el modal no está visible o la imagen es None.

        Args:
            cv_image: Frame BGR como array NumPy ``(H, W, 3)``.
        """
        if cv_image is None or not self.isVisible():
            return
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        qimage = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        raw_pixmap = QPixmap.fromImage(qimage)
        self.cached_pixmap = raw_pixmap.scaled(self.cached_size, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.video_label.setPixmap(self.cached_pixmap)


class AlgorithmResultsWidget(QWidget):
    """Panel horizontal con tarjetas de estado para cada algoritmo de visión.

    Cada tarjeta muestra un valor de texto y un botón que abre el modal
    de stream correspondiente. Algoritmos incluidos: Landolt, QR, Motion, YOLO 3D.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.landolt_dialog = VideoModalDialog("Landolt Detector Stream", self.window())
        self.qr_dialog      = VideoModalDialog("QR Detector Stream",      self.window())
        self.motion_dialog  = VideoModalDialog("Motion Detector Stream",  self.window())
        self.yolo_dialog    = VideoModalDialog("YOLO 3D Stream",          self.window())

        self._setup_ui()

    def _create_card(self, title_text: str, dialog_to_show: VideoModalDialog):
        """Crea una tarjeta estilo GitHub Dark con título y botón de live stream.

        Args:
            title_text: Texto del encabezado de la tarjeta.
            dialog_to_show: Modal que se abre al pulsar el botón.

        Returns:
            Tuple ``(card, value_layout)`` donde ``card`` es el ``QFrame``
            listo para añadir al layout y ``value_layout`` es el
            ``QVBoxLayout`` donde insertar el widget de valor específico.
        """
        card = QFrame()
        card.setMaximumHeight(85)
        card.setStyleSheet("""
            QFrame { background-color: #161B22; border: 1px solid #30363D; border-radius: 8px; }
            QLabel { border: none; }
        """)

        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(5)

        title = QLabel(title_text)
        title.setStyleSheet("color: #C9D1D9; font-weight: bold; font-size: 11px;")

        value_container = QWidget()
        value_layout = QVBoxLayout(value_container)
        value_layout.setContentsMargins(0, 0, 0, 0)

        btn = QPushButton("⏵ Live Stream")
        btn.setStyleSheet("""
            QPushButton {
                background-color: #21262D; color: #8B949E;
                border: 1px solid #363B42; border-radius: 4px; padding: 4px; font-weight: bold; font-size: 8px;
            }
            QPushButton:hover { background-color: #30363D; color: white; }
        """)
        btn.clicked.connect(dialog_to_show.show)

        layout.addWidget(title)
        layout.addWidget(value_container, stretch=1)
        layout.addWidget(btn)

        return card, value_layout

    def _setup_ui(self):
        """Construye el layout horizontal con las cuatro tarjetas de algoritmos."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(15)

        landolt_card, l_layout = self._create_card("⎣ Landolt", self.landolt_dialog)
        self.landolt_label = QLabel("Waiting...")
        self.landolt_label.setStyleSheet("color: #58A6FF; font-size: 22px; font-weight: bold;")
        self.landolt_label.setAlignment(Qt.AlignCenter)
        l_layout.addWidget(self.landolt_label)
        layout.addWidget(landolt_card)

        qr_card, qr_layout = self._create_card("▤ QR Detection", self.qr_dialog)
        self.qr_label = QLabel("Waiting...")
        self.qr_label.setStyleSheet("color: #3FB950; font-size: 16px; font-weight: bold;")
        self.qr_label.setAlignment(Qt.AlignCenter)
        self.qr_label.setWordWrap(True)
        qr_layout.addWidget(self.qr_label)
        layout.addWidget(qr_card)

        motion_card, m_layout = self._create_card("∿ Motion", self.motion_dialog)
        self.motion_status_label = QLabel("UNKNOWN")
        self.motion_status_label.setStyleSheet("color: #8B949E; font-size: 16px; font-weight: bold;")
        self.motion_status_label.setAlignment(Qt.AlignCenter)
        self.motion_area_label = QLabel("Area: 0.0")
        self.motion_area_label.setStyleSheet("color: #8B949E; font-size: 12px;")
        self.motion_area_label.setAlignment(Qt.AlignCenter)
        m_layout.addWidget(self.motion_status_label)
        m_layout.addWidget(self.motion_area_label)
        layout.addWidget(motion_card)

        yolo_card, y_layout = self._create_card("👁 YOLO 3D", self.yolo_dialog)
        self.yolo_label = QLabel("Waiting...")
        self.yolo_label.setStyleSheet("color: #D29922; font-size: 14px; font-weight: bold;")
        self.yolo_label.setAlignment(Qt.AlignCenter)
        self.yolo_label.setWordWrap(True)
        y_layout.addWidget(self.yolo_label)
        layout.addWidget(yolo_card)

    # ------------------------------------------------------------------
    # Slots de actualización de texto / estado
    # ------------------------------------------------------------------

    def update_qr(self, text: str) -> None:
        """Actualiza el texto mostrado en la tarjeta QR."""
        self.qr_label.setText(text)

    def update_landolt(self, orientation: str) -> None:
        """Actualiza la orientación mostrada en la tarjeta Landolt."""
        self.landolt_label.setText(orientation)

    def update_yolo_text(self, text: str) -> None:
        """Actualiza el texto mostrado en la tarjeta YOLO 3D."""
        self.yolo_label.setText(text)

    def update_motion(self, is_stable: bool, area: float) -> None:
        """Actualiza el estado y área de la tarjeta Motion.

        Args:
            is_stable: ``True`` muestra "STABLE" en verde; ``False`` muestra
                "MOTION" en rojo.
            area: Área de movimiento detectada a mostrar.
        """
        if is_stable:
            self.motion_status_label.setText("STABLE")
            self.motion_status_label.setStyleSheet("color: #3FB950; font-size: 16px; font-weight: bold;")
        else:
            self.motion_status_label.setText("MOTION")
            self.motion_status_label.setStyleSheet("color: #F85149; font-size: 16px; font-weight: bold;")
        self.motion_area_label.setText(f"Area: {area:.1f}")

    # ------------------------------------------------------------------
    # Slot de actualización de imagen
    # ------------------------------------------------------------------

    def update_image(self, cv_image: np.ndarray, source_name: str) -> None:
        """Enruta un frame BGR al modal correspondiente según su origen.

        Acepta tanto nombres de topic ROS como nombres amigables.

        Args:
            cv_image: Frame BGR como array NumPy.
            source_name: Topic ROS o nombre amigable del stream. Valores
                soportados: ``/qr_detector/debug_image``, ``QR Video``,
                ``/image``, ``Landolt Video``, ``/motion_detector/debug_image``,
                ``Motion Video``, ``/yolo/dbg_image``, ``YOLO 3D``.
        """
        topic_map = {
            "/qr_detector/debug_image":      self.qr_dialog,
            "QR Video":                      self.qr_dialog,
            "/image":                        self.landolt_dialog,
            "Landolt Video":                 self.landolt_dialog,
            "/motion_detector/debug_image":  self.motion_dialog,
            "Motion Video":                  self.motion_dialog,
            "/yolo/dbg_image":               self.yolo_dialog,
            "YOLO 3D":                       self.yolo_dialog,
        }

        dialog = topic_map.get(source_name)
        if dialog:
            dialog.update_image(cv_image)