import sys
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QGridLayout, QCheckBox, QTableWidget, QTableWidgetItem, QLabel,
    QApplication, QButtonGroup
)
from PyQt6.QtCore import Qt

# ================================
#   GUI
# ================================
class ManualControlWidget(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.previous_robot = None

        # ---------------- 전체 Layout ----------------
        main_layout = QHBoxLayout(self)

        # ---------------- 방향 버튼 ----------------
        left_layout = QGridLayout()
        self.buttons = {}

        self.key_mapping = {
            "u": "↖", "i": "↑", "o": "↗",
            "j": "←", "k": "○", "l": "→",
            "m": "↙", ",": "↓", ".": "↘",
        }

        directions = [
            ("↖", 0, 0), ("↑", 0, 1), ("↗", 0, 2),
            ("←", 1, 0), ("○", 1, 1), ("→", 1, 2),
            ("↙", 2, 0), ("↓", 2, 1), ("↘", 2, 2),
        ]

        for text, r, c in directions:
            btn = QPushButton(text)
            btn.setFixedSize(100, 100)
            btn.clicked.connect(lambda _, t=text: self.handle_button(t))
            self.buttons[text] = btn
            left_layout.addWidget(btn, r, c)

        # 방향키 wrapper (중앙 정렬)
        left_wrapper = QHBoxLayout()
        left_wrapper.addStretch()
        left_wrapper.addLayout(left_layout)
        left_wrapper.addStretch()

        # ---------------- 로봇 선택 + 로그 ----------------
        right_layout = QVBoxLayout()
        robot_names = ["PinkyPro A", "PinkyPro B", "PinkyPro C", "Blackberry"]

        self.robot_id_mapping = {
            "PinkyPro A": 21,
            "PinkyPro B": 22,
            "PinkyPro C": 23,
            "Blackberry": 25
        }

        self.checkboxes = []
        self.button_group = QButtonGroup(self)
        self.button_group.setExclusive(True)

        grid = QGridLayout()
        for i, name in enumerate(robot_names):
            cb = QCheckBox(name)
            cb.clicked.connect(lambda _, c=cb: self.on_checkbox_clicked(c))
            self.checkboxes.append(cb)
            self.button_group.addButton(cb)
            grid.addWidget(cb, i // 2, i % 2)

        right_layout.addLayout(grid)

        # 로그 테이블
        self.log_table = QTableWidget(0, 3)
        self.log_table.setHorizontalHeaderLabels(["로봇 ID", "로봇 이름", "로그"])
        self.log_table.setColumnWidth(0, 80)
        self.log_table.setColumnWidth(1, 120)
        self.log_table.setColumnWidth(2, 175)
        right_layout.addWidget(QLabel("로봇 로그"))
        right_layout.addWidget(self.log_table)

        # 오른쪽 wrapper
        right_wrapper = QHBoxLayout()
        right_wrapper.addLayout(right_layout)

        # ---------------- layout 배치 ----------------
        main_layout.addLayout(left_wrapper, 1)
        main_layout.addLayout(right_wrapper, 1)

    # ---------------- 로봇 선택 ----------------
    def on_checkbox_clicked(self, cb):
        if cb.isChecked():
            if self.previous_robot and self.previous_robot != cb:
                prev_name = self.previous_robot.text()
                prev_id = self.robot_id_mapping[prev_name]
                self.add_log(prev_id, prev_name, "disconnect")

            name = cb.text()
            rid = self.robot_id_mapping[name]
            self.add_log(rid, name, "connecting")
            self.previous_robot = cb
        else:
            name = cb.text()
            rid = self.robot_id_mapping[name]
            self.add_log(rid, name, "disconnect")
            self.previous_robot = None

    # ---------------- 버튼 클릭 ----------------
    def handle_button(self, direction):
        for cb in self.checkboxes:
            if cb.isChecked():
                name = cb.text()
                rid = self.robot_id_mapping[name]
                self.add_log(rid, name, f"{direction} 명령")

    # ---------------- 로그 ----------------
    def add_log(self, rid, name, log_msg):
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        self.log_table.setItem(row, 0, QTableWidgetItem(str(rid)))
        self.log_table.setItem(row, 1, QTableWidgetItem(name))
        self.log_table.setItem(row, 2, QTableWidgetItem(log_msg))


# ================================
#           MAIN
# ================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = ManualControlWidget()
    widget.show()
    sys.exit(app.exec())
