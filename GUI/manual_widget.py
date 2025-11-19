from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QGridLayout, QCheckBox, QTableWidget, QTableWidgetItem, QLabel
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import Qt


class ManualControlWidget(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        # ---------------- 전체 레이아웃 ----------------
        main_layout = QHBoxLayout(self)

        # ============================================================
        # 1) 왼쪽 영역: 방향(3x3) 버튼
        # ============================================================
        left_layout = QGridLayout()
        self.buttons = {}

        # 방향 버튼 정의
        directions = [
            ("↖", 0, 0), ("↑", 0, 1), ("↗", 0, 2),
            ("←", 1, 0), ("○", 1, 1), ("→", 1, 2),
            ("↙", 2, 0), ("↓", 2, 1), ("↘", 2, 2),
        ]

        # 버튼 생성
        for text, row, col in directions:
            btn = QPushButton(text)
            btn.setFixedSize(80, 80)
            btn.clicked.connect(lambda _, t=text: self.handle_button(t))
            self.buttons[text] = btn
            left_layout.addWidget(btn, row, col)

        # ============================================================
        # 2) 오른쪽 영역: 체크박스(로봇 선택) + 로그
        # ============================================================
        right_layout = QVBoxLayout()

        # --- 2x2 체크박스: 로봇 선택 ---
        robot_names = ["로봇A", "로봇B", "로봇C", "로봇D"]
        self.checkboxes = []
        grid = QGridLayout()

        for i, name in enumerate(robot_names):
            cb = QCheckBox(name)
            self.checkboxes.append(cb)
            grid.addWidget(cb, i // 2, i % 2)

        right_layout.addLayout(grid)

        # --- 로그 테이블 ---
        self.log_table = QTableWidget(0, 3)
        self.log_table.setHorizontalHeaderLabels(["로봇 ID", "로봇 이름", "로그명"])
        self.log_table.setColumnWidth(0, 80)
        self.log_table.setColumnWidth(1, 120)
        self.log_table.setColumnWidth(2, 200)

        right_layout.addWidget(QLabel("로봇 로그"))
        right_layout.addWidget(self.log_table)

        # ---------------- 최종 레이아웃 결합 ----------------
        left_container = QWidget()
        left_container.setLayout(left_layout)
        right_container = QWidget()
        right_container.setLayout(right_layout)

        right_container.setFixedWidth(420)  # 오른쪽을 420px로 고정

        main_layout.addWidget(left_container)
        main_layout.addWidget(right_container)

        main_layout.setStretch(0, 1)
        main_layout.setStretch(1, 1)

    # ============================================================
    # 버튼 클릭 시, 체크된 로봇에 대해 로그 남기기
    # ============================================================
    def handle_button(self, direction):
        """방향 버튼 클릭 처리"""
        for cb in self.checkboxes:
            if cb.isChecked():
                robot_name = cb.text()
                robot_id = self.get_robot_id(robot_name)
                log_msg = f"{direction} 명령 수행중"
                self.add_log(robot_id, robot_name, log_msg)

    # --- 로봇 이름 → ID 매핑 ---
    def get_robot_id(self, name):
        mapping = {
            "로봇A": 1,
            "로봇B": 2,
            "로봇C": 3,
            "로봇D": 4
        }
        return mapping.get(name, 0)

    # --- 로그 추가 ---
    def add_log(self, robot_id, robot_name, log_msg):
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        self.log_table.setItem(row, 0, QTableWidgetItem(str(robot_id)))
        self.log_table.setItem(row, 1, QTableWidgetItem(robot_name))
        self.log_table.setItem(row, 2, QTableWidgetItem(log_msg))
