from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QCheckBox, QPushButton, QTableWidget, QTableWidgetItem, QHeaderView
)
from PySide6.QtCore import Qt
from datetime import datetime

class EventLogWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QWidget { background-color: #161B22; border: 1px solid #30363D; border-radius: 8px; }
            QLabel { border: none; }
        """)
        self._setup_ui()
        
        # Agregamos algunos mensajes de prueba iniciales
        self.log_event("INFO", "System", "Sentinel Console Initialized")
        self.log_event("INFO", "ROS 2", "Waiting for topics...")

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 10, 15, 15)
        layout.setSpacing(10)

        # --- CONTROLES SUPERIORES ---
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

        # --- TABLA DE LOGS ---
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Time", "Level", "Source", "Message"])
        self.table.verticalHeader().setVisible(False)
        self.table.setShowGrid(False)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        
        # Estilo oscuro para la tabla
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #0D1117;
                color: #C9D1D9;
                border: 1px solid #30363D;
                border-radius: 4px;
                font-size: 11px;
            }
            QHeaderView::section {
                background-color: #161B22;
                color: #8B949E;
                font-weight: bold;
                border: none;
                border-bottom: 1px solid #30363D;
                padding: 4px;
            }
            QScrollBar:vertical {
                border: none;
                background: #0D1117;
                width: 10px;
            }
            QScrollBar::handle:vertical {
                background: #30363D;
                border-radius: 5px;
                min-height: 20px;
            }
        """)

        # Ajustar anchos de columna (Fijo para Time, Level y Source. Message ocupa el resto)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Fixed)
        self.table.setColumnWidth(0, 90) # Time
        header.setSectionResizeMode(1, QHeaderView.Fixed)
        self.table.setColumnWidth(1, 60) # Level
        header.setSectionResizeMode(2, QHeaderView.Fixed)
        self.table.setColumnWidth(2, 90) # Source
        header.setSectionResizeMode(3, QHeaderView.Stretch) # Message

        layout.addWidget(self.table)

    def log_event(self, level, source, message):
        """Agrega una nueva fila al log"""
        row = self.table.rowCount()
        self.table.insertRow(row)

        # Tiempo actual
        time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        time_item = QTableWidgetItem(time_str)
        time_item.setTextAlignment(Qt.AlignCenter)
        self.table.setItem(row, 0, time_item)

        # Etiqueta de Nivel (INFO, WARN, ERROR) con colores
        lbl_level = QLabel(level)
        lbl_level.setAlignment(Qt.AlignCenter)
        if level == "INFO":
            lbl_level.setStyleSheet("background-color: #1F6FEB; color: white; border-radius: 3px; font-weight: bold; font-size: 10px; margin: 2px;")
        elif level == "WARN":
            lbl_level.setStyleSheet("background-color: #D29922; color: white; border-radius: 3px; font-weight: bold; font-size: 10px; margin: 2px;")
        elif level == "ERROR":
            lbl_level.setStyleSheet("background-color: #F85149; color: white; border-radius: 3px; font-weight: bold; font-size: 10px; margin: 2px;")
        self.table.setCellWidget(row, 1, lbl_level)

        # Fuente y Mensaje
        source_item = QTableWidgetItem(source)
        source_item.setTextAlignment(Qt.AlignCenter)
        self.table.setItem(row, 2, source_item)

        msg_item = QTableWidgetItem(message)
        self.table.setItem(row, 3, msg_item)
        
        # Ajustar la altura de la fila para que sea compacta
        self.table.setRowHeight(row, 24)

        if self.auto_scroll_cb.isChecked():
            self.table.scrollToBottom()

    def clear_logs(self):
        self.table.setRowCount(0)