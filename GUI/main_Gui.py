from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QStackedWidget, QLabel, QHBoxLayout
from PyQt6.QtWidgets import QTableWidget, QTableWidgetItem
import sys
import cv2
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QPixmap, QPainter, QColor, QPolygonF
from PyQt6.QtCore import QPointF
from PyQt6.QtCore import Qt
from datetime import datetime


class MapWidget(QLabel):
    def __init__(self, map_path):
        super().__init__()
        self.base_pixmap = QPixmap(map_path)
        self.setPixmap(self.base_pixmap)
        self.robots = {}  # domain_id -> (x,y)

    def set_robot(self, domain_id, x, y):
        self.robots[domain_id] = (x, y)
        self.repaint()

    def paintEvent(self, event):
        painter = QPainter(self)
        scaled = self.base_pixmap.scaled(
            self.width(), self.height(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        painter.drawPixmap(0, 0, scaled)

        for domain, (x, y) in self.robots.items():
            # world 좌표 → 픽셀 변환 (간단히 스케일링 예시)
            px = int(x * 10)
            py = int(y * 10)
            # 삼각형 그리기
            size = 10
            tri = QPolygonF([
                QPointF(px, py - size),
                QPointF(px - size, py + size),
                QPointF(px + size, py + size)
            ])
            painter.setBrush(QColor(255, 0, 0))
            painter.drawPolygon(tri)
            painter.drawText(px+12, py, str(domain))
        painter.end()

class CameraWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.label = QLabel("카메라 화면")
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)

    def start_camera(self):
        # 이미 타이머가 돌고 있으면 아무 것도 안 함
        if self.timer.isActive():
            return
        # cap이 없거나 열려있지 않으면 연다
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)
        # cap이 정상 열렸을 때만 타이머 시작
        if self.cap is not None and self.cap.isOpened():
            self.timer.start(30)

    def stop_camera(self):
        # 타이머 중지 및 cap 해제 (안전하게)
        if self.timer.isActive():
            self.timer.stop()
        if self.cap is not None:
            try:
                if self.cap.isOpened():
                    self.cap.release()
            except Exception:
                pass
            self.cap = None
        # 화면 지우기
        self.label.clear()

    def update_frame(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        img = QImage(frame.data, w, h, ch * w, QImage.Format.Format_RGB888)
        self.label.setPixmap(QPixmap.fromImage(img))
            
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("찬LOGIC")

        # # 레이아웃 최소/최대 크기 지정
        # self.setMinimumSize(640, 480)
        # self.setMaximumSize(1280, 1024)
        self.setFixedSize(800, 600) # 창 크기 지정

        self.title_label = QLabel("메인 상태 화면")
        self.title_label.setFixedHeight(30)

        # 상단 버튼
        self.button_main = QPushButton("메인 화면")
        self.button_io = QPushButton("입/출고")
        self.button_staff = QPushButton("임직원")
        self.button_cctv = QPushButton("CCTV")
        self.button_manual = QPushButton("수동 조작")

        for b in (self.button_main, self.button_io, self.button_staff,
                  self.button_cctv, self.button_manual):
            b.setFixedSize(120, 40)

        # --- 메인 화면 구성: 맵 + 로그 ---
        self.map_widget = MapWidget("/home/addinedu/dev_ws/ros-repo-2/GUI/map.png")
        self.log_table = QTableWidget()
        self.log_table.setColumnCount(3)
        self.log_table.setHorizontalHeaderLabels(["ID", "좌표(x, y)", "수신 일시"])
        self.log_table.setMinimumWidth(355)
        self.log_table.setColumnWidth(0, 50)
        self.log_table.setColumnWidth(1, 150)
        self.log_table.setColumnWidth(2, 150)

        content_layout = QHBoxLayout()
        content_layout.addWidget(self.map_widget, stretch=2)
        content_layout.addWidget(self.log_table, stretch=3)

        self.main_page = QWidget()
        self.main_page.setLayout(content_layout)

        # --- 스택 구성 ---
        self.stack = QStackedWidget()
        self.stack.addWidget(self.main_page)               # index 0
        self.stack.addWidget(QLabel("입/출고 화면"))       # index 1
        self.stack.addWidget(QLabel("임직원 화면"))       # index 2
        self.cctv_widget = CameraWidget()                 # index 3
        self.stack.addWidget(self.cctv_widget)
        self.stack.addWidget(QLabel("수동 조작"))         # index 4

        self.stack.currentChanged.connect(self.on_stack_changed)

        # 버튼 이벤트 연결
        self.button_main.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        self.button_io.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        self.button_staff.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        self.button_cctv.clicked.connect(lambda: self.stack.setCurrentIndex(3))
        self.button_manual.clicked.connect(lambda: self.stack.setCurrentIndex(4))

        # 레이아웃 구성
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_main)
        button_layout.addWidget(self.button_io)
        button_layout.addWidget(self.button_staff)
        button_layout.addWidget(self.button_cctv)
        button_layout.addWidget(self.button_manual)

        main_layout = QVBoxLayout()
        main_layout.addLayout(button_layout)
        main_layout.addWidget(self.stack)
        main_layout.addWidget(self.title_label)

        self.setLayout(main_layout)

        # 초기 화면을 메인으로
        self.stack.setCurrentIndex(0)
    
    def on_stack_changed(self, idx: int):
        # CCTV 인덱스(여기선 3)에 들어오면 start, 아니면 stop
        current = self.stack.currentWidget()
        if current is self.cctv_widget:
            self.title_label.setText("CCTV 화면")
            self.cctv_widget.start_camera()

        elif isinstance(current, QLabel) and current.text() == "메인 화면":
            self.title_label.setText("메인 상태 화면")
            self.cctv_widget.stop_camera()


        elif isinstance(current, QLabel) and current.text() == "입/출고 화면":
            self.title_label.setText("입/출고")
            self.cctv_widget.stop_camera()

        elif isinstance(current, QLabel) and current.text() == "임직원 화면":
            self.title_label.setText("임직원")
            self.cctv_widget.stop_camera()

        elif isinstance(current, QLabel) and current.text() == "수동 조작":
            self.title_label.setText("수동 조작")
            self.cctv_widget.stop_camera()

        elif current is self.map_widget:
            self.title_label.setText("메인 상태 화면")
            self.cctv_widget.stop_camera()

    def add_log(self, domain_id, x, y):
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        self.log_table.setItem(row, 0, QTableWidgetItem(str(domain_id)))
        self.log_table.setItem(row, 1, QTableWidgetItem(f"({x:.2f}, {y:.2f})"))
        self.log_table.setItem(row, 2, QTableWidgetItem(datetime.now().strftime("%H:%M:%S")))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())