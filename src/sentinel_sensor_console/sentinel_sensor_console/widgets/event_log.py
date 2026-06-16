from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QCheckBox, QPushButton, QTableWidget, QTableWidgetItem, QHeaderView
)
from PySide6.QtCore import Qt
from datetime import datetime


class EventLogWidget(QWidget):
    """Tabla de log de eventos con niveles INFO / WARN / ERROR.

    Muestra columnas Time, Level, Source y Message. La columna Level
    usa un ``QLabel`` con badge de color en lugar de texto plano.
    Incluye checkbox de auto-scroll y botón de limpieza.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QWidget { background-color: #161B22; border: 1px solid #30363D; border-radius: 8px; }
            QLabel { border: none; }
        """)
        self._setup_ui()

        self.log_event("INFO", "System", "Sentinel Console Initialized")
        self.log_event("INFO", "ROS 2", "Waiting for topics...")

    def _setup_ui(self):
        """Construye el header con controles y la tabla de logs."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 10, 15, 15)
        layout.setSpacing(10)

        # --- Controles superiores ---
        header_layout = QHBoxLayout()

        title = QLabel("EVENT LOG")
        title.setStyleSheet("color: #8B949E; font-weight: bold; font-size: 12px;")

        self.auto_scroll_cb = QCheckBox("Auto scroll")
        self.auto_scroll_cb.setChecked(True)
        self.auto_scroll_cb.setStyleSheet("color: #C9D1D9; font-size: 11px; border: none;")

        btn_clear = QPushButton("Clear Log")
        btn_clear.setStyleSheet("""
            QPushButton {
                background-color: #21262D; color: #C9D1D9;
                border: 1px solid #363B42; border-radius: 4px; padding: 4px 10px; font-size: 11px;
            }
            QPushButton:hover { background-color: #30363D; }
        """)
        btn_clear.clicked.connect(self.clear_logs)

        header_layout.addWidget(title)
        header_layout.addStretch()
        header_layout.addWidget(self.auto_scroll_cb)
        header_layout.addWidget(btn_clear)
        layout.addLayout(header_layout)

        # --- Tabla ---
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Time", "Level", "Source", "Message"])
        self.table.verticalHeader().setVisible(False)
        self.table.setShowGrid(False)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #0D1117; color: #C9D1D9;
                border: 1px solid #30363D; border-radius: 4px; font-size: 11px;
            }
            QHeaderView::section {
                background-color: #161B22; color: #8B949E;
                font-weight: bold; border: none;
                border-bottom: 1px solid #30363D; padding: 4px;
            }
            QScrollBar:vertical { border: none; background: #0D1117; width: 10px; }
            QScrollBar::handle:vertical { background: #30363D; border-radius: 5px; min-height: 20px; }
        """)

        # Time, Level, Source fijos; Message ocupa el resto
        header = self.table.horizontalHeader()
        for col, width in [(0, 90), (1, 60), (2, 90)]:
            header.setSectionResizeMode(col, QHeaderView.Fixed)
            self.table.setColumnWidth(col, width)
        header.setSectionResizeMode(3, QHeaderView.Stretch)

        layout.addWidget(self.table)

    def log_event(self, level: str, source: str, message: str) -> None:
        """Inserta una fila al final de la tabla con timestamp actual.

        La columna Level muestra un badge ``QLabel`` con color según nivel:

        - ``INFO``  → azul  ``#1F6FEB``
        - ``WARN``  → ámbar ``#D29922``
        - ``ERROR`` → rojo  ``#F85149``

        Hace scroll al fondo si auto-scroll está activo.

        Args:
            level: Nivel del evento. Valores esperados: ``"INFO"``, ``"WARN"``, ``"ERROR"``.
            source: Nombre del componente o nodo que genera el evento.
            message: Descripción del evento.
        """
        row = self.table.rowCount()
        self.table.insertRow(row)

        time_item = QTableWidgetItem(datetime.now().strftime("%H:%M:%S.%f")[:-3])
        time_item.setTextAlignment(Qt.AlignCenter)
        self.table.setItem(row, 0, time_item)

        lbl_level = QLabel(level)
        lbl_level.setAlignment(Qt.AlignCenter)
        color_map = {
            "INFO":  "#1F6FEB",
            "WARN":  "#D29922",
            "ERROR": "#F85149",
        }
        bg = color_map.get(level, "#30363D")
        lbl_level.setStyleSheet(
            f"background-color: {bg}; color: white; border-radius: 3px; "
            "font-weight: bold; font-size: 10px; margin: 2px;"
        )
        self.table.setCellWidget(row, 1, lbl_level)

        source_item = QTableWidgetItem(source)
        source_item.setTextAlignment(Qt.AlignCenter)
        self.table.setItem(row, 2, source_item)

        self.table.setItem(row, 3, QTableWidgetItem(message))
        self.table.setRowHeight(row, 24)

        if self.auto_scroll_cb.isChecked():
            self.table.scrollToBottom()

    def clear_logs(self) -> None:
        """Elimina todas las filas de la tabla."""
        self.table.setRowCount(0)