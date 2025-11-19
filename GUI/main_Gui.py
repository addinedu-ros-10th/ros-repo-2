import sys
import threading
from datetime import datetime
import pandas as pd

import cv2
import rclpy
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QImage, QPixmap, QPainter, QColor, QPolygonF
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QStackedWidget,
    QTableWidget, QTableWidgetItem
)
from PyQt6.QtWidgets import (
    QLineEdit, QSpinBox, QCheckBox, QTextEdit, QGroupBox, QFormLayout,
    QHeaderView
)
from PyQt6.QtCore import pyqtSignal, QObject

from GUI.signaller import BridgeSignaller
from server.Central_control import ROSTCPBridge
from GUI.io_widget import IOWidget
from GUI.staff_widget import StaffWidget

# ------------------------- [지도 위젯] -------------------------
class MapWidget(QLabel):
    """로봇의 위치를 지도 위에 표시하는 위젯"""

    def __init__(self, map_path: str):
        super().__init__()
        self.base_pixmap = QPixmap(map_path)
        self.setPixmap(self.base_pixmap)
        self.robots = {}  # domain_id → (x, y)

    def set_robot(self, domain_id: int, x: float, y: float):
        """로봇의 위치 업데이트"""
        self.robots[domain_id] = (x, y)
        self.repaint()

    def paintEvent(self, event):
        painter = QPainter(self)
         # 원본 이미지를 비율 유지하면서 위젯 크기에 맞게 스케일링
        scaled = self.base_pixmap.scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        # 중앙 정렬을 위한 오프셋 계산
        x = (self.width() - scaled.width()) // 2
        y = (self.height() - scaled.height()) // 2

        # 중앙에 그리기
        painter.drawPixmap(x, y, scaled)

        # 로봇 좌표 표시 (스케일 비율 반영)
        scale_x = scaled.width() / self.base_pixmap.width()
        scale_y = scaled.height() / self.base_pixmap.height()

        for domain, (x_val, y_val) in self.robots.items():
            px = int(x_val * 10 * scale_x) + x
            py = int(y_val * 10 * scale_y) + y

            size = 10
            tri = QPolygonF([
                QPointF(px, py - size),
                QPointF(px - size, py + size),
                QPointF(px + size, py + size)
            ])
            painter.setBrush(QColor(255, 0, 0))
            painter.drawPolygon(tri)
            painter.drawText(px + 12, py, str(domain))

        painter.end()


# ------------------------- [카메라 위젯] -------------------------
class CameraWidget(QWidget):
    """웹캠 영상을 표시하는 위젯"""

    def __init__(self):
        super().__init__()
        self.label = QLabel("카메라 화면")
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

    def start_camera(self):
        """카메라 시작"""
        if self.timer.isActive():
            return
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture("http://192.168.2.100:81/stream")
            # ESP32-CAM 해상도 설정
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if self.cap and self.cap.isOpened():
            self.timer.start(30)

    def stop_camera(self):
        """카메라 정지"""
        if self.timer.isActive():
            self.timer.stop()
        if self.cap:
            try:
                if self.cap.isOpened():
                    self.cap.release()
            except Exception:
                pass
            self.cap = None
        self.label.clear()

    def update_frame(self):
        """프레임 갱신"""
        if not self.cap:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        img = QImage(frame.data, w, h, ch * w, QImage.Format.Format_RGB888)

        pixmap = QPixmap.fromImage(img)

        # QLabel 크기에 맞게 비율 유지하며 스케일링
        scaled_pixmap = pixmap.scaled(
            self.label.size(),              # 현재 QLabel 크기
            Qt.AspectRatioMode.KeepAspectRatio,      # 비율 유지
            Qt.TransformationMode.SmoothTransformation  # 부드럽게 확대/축소
        )

        self.label.setPixmap(scaled_pixmap)
        
# ------------------------- [메인 윈도우] -------------------------
class MainWindow(QWidget):
    """메인 GUI: 지도, 로그, 버튼, CCTV 등 통합"""

    def __init__(self, signaller):
        super().__init__()
        self.signaller = signaller
        self.setWindowTitle("찬LOGIC")
        self.setFixedSize(800, 600)

        self.title_label = QLabel("메인 상태 화면")
        self.title_label.setFixedHeight(30)

        # --- 버튼 생성 ---
        self.button_main = QPushButton("메인 화면")
        self.button_io = QPushButton("입/출고")
        self.button_storage = QPushButton("제품 현황")
        self.button_staff = QPushButton("임직원")
        self.button_cctv = QPushButton("CCTV")
        self.button_manual = QPushButton("수동 조작")

        for btn in (self.button_main, self.button_io, self.button_storage, self.button_staff,
                    self.button_cctv, self.button_manual):
            btn.setFixedSize(120, 40)

        # --- 메인 페이지 구성 ---
        self.map_widget = MapWidget("/home/addinedu/dev_ws/ros-repo-2/GUI/map.png")
        # self.map_widget.setMaximumWidth(400)
        self.log_table = QTableWidget(0, 3)
        self.log_table.setHorizontalHeaderLabels(["ID", "좌표(x, y)", "수신 일시"])
        self.log_table.setMinimumWidth(360)
        self.log_table.setColumnWidth(0, 50)
        self.log_table.setColumnWidth(1, 150)
        self.log_table.setColumnWidth(2, 150)

        content_layout = QHBoxLayout()
        content_layout.addWidget(self.map_widget, 2)
        content_layout.addWidget(self.log_table, 3)

        main_page = QWidget()
        main_page.setLayout(content_layout)

        # --- 스택 구성 ---
        self.stack = QStackedWidget()
        self.stack.addWidget(main_page)                  # index 0
        # self.stack.addWidget(QLabel("입/출고 화면"))      # index 1
        self.io_widget = IOWidget(signaller)   # signaller는 main 실행부에서 만든 객체
        self.stack.addWidget(self.io_widget)   # index 1
        self.stack.addWidget(QLabel("제품 현황"))      # index 2
        self.staff_widget = StaffWidget(signaller)   # index 3
        self.stack.addWidget(self.staff_widget)
        # self.stack.addWidget(QLabel("임직원 화면"))      # index 3
        self.cctv_widget = CameraWidget()                # index 4
        self.stack.addWidget(self.cctv_widget)
        self.stack.addWidget(QLabel("수동 조작"))        # index 5

        # --- 버튼 이벤트 연결 ---
        self.button_main.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        self.button_io.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        self.button_storage.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        self.button_staff.clicked.connect(lambda: self.stack.setCurrentIndex(3))
        self.button_cctv.clicked.connect(lambda: self.stack.setCurrentIndex(4))
        self.button_manual.clicked.connect(lambda: self.stack.setCurrentIndex(5))
        self.stack.currentChanged.connect(self.on_stack_changed)

        # --- 메인 레이아웃 ---
        button_layout = QHBoxLayout()
        for btn in (self.button_main, self.button_io, self.button_storage, self.button_staff,
                    self.button_cctv, self.button_manual):
            button_layout.addWidget(btn)

        layout = QVBoxLayout()
        layout.addLayout(button_layout)
        layout.addWidget(self.stack)
        layout.addWidget(self.title_label)
        self.setLayout(layout)

        # 기본 화면 설정
        self.stack.setCurrentIndex(0)

    # -------------------- 화면 전환 이벤트 --------------------
    def on_stack_changed(self, idx: int):
        current = self.stack.currentWidget()

        if current is self.cctv_widget:
            self.title_label.setText("CCTV 화면")
            self.cctv_widget.start_camera()
        else:
            self.cctv_widget.stop_camera()
            text_map = {
                0: "메인 상태 화면",
                1: "입/출고 로그",
                2: "제품 현황",
                3: "임직원 현황",
                5: "수동 조작"
            }
            self.title_label.setText(text_map.get(idx, ""))

    # -------------------- 로그 추가 --------------------
    def add_log(self, domain_id: int, x: float, y: float):
        """로봇 좌표 수신 시 테이블에 로그 추가"""
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        self.log_table.setItem(row, 0, QTableWidgetItem(str(domain_id)))
        self.log_table.setItem(row, 1, QTableWidgetItem(f"({x:.2f}, {y:.2f})"))
        self.log_table.setItem(row, 2, QTableWidgetItem(datetime.now().strftime("%H:%M:%S")))


# ------------------------- [메인 실행부] -------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # 1) signaller 생성 (기존)
    signaller = BridgeSignaller()

    # 2) MainWindow에 signaller 전달
    window = MainWindow(signaller)
    window.show()

    # 3) 시그널 연결: ROS → GUI
    def update_gui(domain_id, x, y):
        window.map_widget.set_robot(domain_id, x, y)
        window.add_log(domain_id, x, y)

    signaller.robot_signal.connect(update_gui)

    # 4) (선택) 입/출고 로그가 개별 연결이 필요하면 여기서 연결 가능
    # 예: signaller.io_logs_signal.connect(window.io_widget.update_logs)
    window.io_widget.set_products(["선택하세요.", "화장품", "제품B", "제품C"])
    if hasattr(signaller, "io_logs_signal"):
        try:
            signaller.io_logs_signal.connect(window.io_widget.update_logs)
        except Exception:
            pass

    # 5) staff_list_add 판다스 프레임 저장용 시그널 연결
    df = pd.DataFrame(columns=["name", "phone", "date", "uid"])

    def update_staff_list(new_entry):
        global df
        df = pd.concat([df, pd.DataFrame([new_entry])], ignore_index=True)

        # StaffWidget 테이블 갱신 호출
        window.staff_widget.update_log_table(df)

    if hasattr(signaller, "staff_list_add"):
        try:
            signaller.staff_list_add.connect(update_staff_list)
            
        except Exception:
            pass
    
    def delete_staff_row(row):
        global df
        if 0 <= row < len(df):
            df = df.drop(index=row).reset_index(drop=True)
            print(f"GUI: 직원 데이터프레임에서 행 {row} 삭제\n", df)

            # StaffWidget 테이블 갱신 호출
            window.staff_widget.update_log_table(df)

    if hasattr(signaller, "staff_delete_row"):
        try:
            signaller.staff_delete_row.connect(delete_staff_row)
        except Exception:
            pass

    # ROS2 노드 스레드 실행
    def ros_thread():
        rclpy.init()
        node = ROSTCPBridge(signaller)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.stop_flag = True
            node.destroy_node()
            rclpy.shutdown()

    threading.Thread(target=ros_thread, daemon=True).start()
    sys.exit(app.exec())
