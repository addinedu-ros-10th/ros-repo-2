import sys
import threading
from datetime import datetime

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QStackedWidget,
    QTableWidget, QTableWidgetItem
)
from PyQt6.QtWidgets import (
    QLineEdit, QSpinBox, QCheckBox, QTextEdit, QGroupBox, QFormLayout,
    QHeaderView, QComboBox, QHBoxLayout, QMessageBox, QDialog, QDateEdit
)
from PyQt6.QtCore import pyqtSignal, QObject, QStringListModel, QTimer, QDate
from PyQt6.QtGui import QStandardItemModel, QStandardItem
import pandas as pd


class StaffWidget(QWidget):
    def __init__(self, signaller):
        super().__init__()
        self.signaller = signaller
        self.waiting_for_rfid = False   # 등록 대기 상태 플래그

        self.name_edit = QLineEdit()
        self.phone_edit = QLineEdit()
        self.date_edit = QDateEdit()
        self.date_edit.setCalendarPopup(True)
        self.date_edit.setDate(QDate.currentDate())  # 오늘 날짜로 초기화

        self.register_button = QPushButton("등록")
        self.register_button.clicked.connect(self.on_register)

        self.delete_button = QPushButton("삭제")
        self.delete_button.clicked.connect(self.on_delete)  # 삭제 처리 함수 연결

        # RFID 신호 연결
        self.signaller.staff_rfid_signal_1.connect(self.on_rfid_received)
        
        # 버튼 가로로 배치
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.register_button)
        button_layout.addWidget(self.delete_button)

        # 레이아웃 설정
        form = QFormLayout()
        form.addRow("이름:", self.name_edit)
        form.addRow("핸드폰(뒤 4자리):", self.phone_edit)
        form.addRow("입사일:", self.date_edit)
        form.addRow("", button_layout)

        self.log_table = QTableWidget(0, 4)
        self.log_table.setHorizontalHeaderLabels(["이름", "핸드폰", "입사일", "카드번호"])
        # self.df = pd.DataFrame(columns=["name", "phone", "date", "uid"])

        self.log_table.setColumnWidth(0, 80)   # 첫 번째 열(ID)
        self.log_table.setColumnWidth(1, 200)  # 두 번째 열(좌표)
        self.log_table.setColumnWidth(2, 260)  # 세 번째 열(시간)
        self.log_table.setColumnWidth(3, 210)  # 네 번째 열(카드번호)

        layout = QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(self.log_table)
        self.setLayout(layout)

        self.pending_dialog = None

    def on_register(self):
        self.pending_dialog = QMessageBox(self)
        msg = self.pending_dialog
        
        if not self.name_edit.text() or not self.phone_edit.text():
            msg.setWindowTitle("오류")
            msg.setText("이름과 핸드폰 번호를 입력해주세요.")
            msg.setStandardButtons(QMessageBox.StandardButton.Ok)
            msg.exec()
            return
        # self.phone_edit에서 숫자 4자리가 아닐경우 오류 처리
        if not self.phone_edit.text().isdigit() or len(self.phone_edit.text()) != 4:
            msg.setWindowTitle("오류")
            msg.setText("핸드폰 뒤 4자리를 입력해주세요.")
            msg.setStandardButtons(QMessageBox.StandardButton.Ok)
            msg.exec()
            return

        self.waiting_for_rfid = True
        msg.setWindowTitle("RFID 등록")
        msg.setText("RFID 카드를 인식해주세요")
        msg.setStandardButtons(QMessageBox.StandardButton.Cancel)
        result = msg.exec()

        if result == QMessageBox.StandardButton.Cancel:
            cancel_msg = QMessageBox(self)
            cancel_msg.setWindowTitle("알림")
            cancel_msg.setText("취소되었습니다.")
            cancel_msg.setStandardButtons(QMessageBox.StandardButton.Ok)  # 확인 버튼 추가
            cancel_msg.show()  # 확인 누르면 닫힘
            self.waiting_for_rfid = False
            self.pending_dialog = None

    def on_rfid_received(self, uid):

        # 등록 대기 상태일 때만 처리
        if not self.waiting_for_rfid:
            return
        
        # RFID 수신 시 알림창 닫고 로그 추가
        if self.pending_dialog:
            self.pending_dialog.done(QMessageBox.StandardButton.Ok) # 강제로 닫기
            self.pending_dialog = None

        QMessageBox.information(self, "등록 완료", "등록처리가 완료되었습니다.")
        self.waiting_for_rfid = False

        name = self.name_edit.text()
        phone = self.phone_edit.text()[-4:]  # 뒤 4자리
        date = self.date_edit.date().toString("yyyy-MM-dd")

        # GUI에서 서버로 직원 데이터 전달
        df = {"name": name, "phone": phone, "date": date, "uid": uid}
        print(df)
        self.signaller.staff_list_add.emit(df)
        print("GUI: staff_list_add 시그널 발행 완료", df)

        # row = self.log_table.rowCount()
        # self.log_table.insertRow(row)
        # self.log_table.setItem(row, 0, QTableWidgetItem(name))
        # self.log_table.setItem(row, 1, QTableWidgetItem(phone))
        # self.log_table.setItem(row, 2, QTableWidgetItem(date))
        # self.log_table.setItem(row, 3, QTableWidgetItem(uid))

    def on_delete(self):
        selected_rows = set()
        for item in self.log_table.selectedItems():
            selected_rows.add(item.row())

        for row in sorted(selected_rows, reverse=True):
            # 삭제할 이름 가져오기 (첫 번째 열 기준)
            name_item = self.log_table.item(row, 0)
            name = name_item.text() if name_item else "선택된 항목"

            # 확인/취소 메시지 박스
            reply = QMessageBox.question(
                self,
                "삭제 확인",
                f"{name}을(를) 삭제하시겠습니까?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
            )

            if reply == QMessageBox.StandardButton.Yes:
                self.log_table.removeRow(row)


    def update_log_table(self, staff_list):
        # staff_list = [(name, phone, date, uid), ...]
        print("수신 완료")
        self.log_table.setRowCount(0)  # 기존 내용 지우기
        for _, row in staff_list.iterrows():
            r = self.log_table.rowCount()
            self.log_table.insertRow(r)
            self.log_table.setItem(r, 0, QTableWidgetItem(row["name"]))
            self.log_table.setItem(r, 1, QTableWidgetItem(row["phone"]))
            self.log_table.setItem(r, 2, QTableWidgetItem(row["date"]))
            self.log_table.setItem(r, 3, QTableWidgetItem(row["uid"]))