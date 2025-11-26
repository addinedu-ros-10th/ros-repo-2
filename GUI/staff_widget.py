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
        self.mode = None  # "register" or "delete"

         # 삭제용 임시 저장공간
        self.delete_target_row = None
        self.delete_target_uid = None

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
        self.mode = "register"
        
        
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
        
        # 강제로 열려 있는 대화상자 닫기
        if self.pending_dialog:
            try:
                self.pending_dialog.done(QMessageBox.StandardButton.Ok)
            except Exception:
                pass
            self.pending_dialog = None

        if self.mode == "register":
            self.waiting_for_rfid = False
            self.mode = None
            QMessageBox.information(self, "등록 완료", "등록처리가 완료되었습니다.")

            name = self.name_edit.text()
            phone = self.phone_edit.text()
            date = self.date_edit.date().toString("yyyy-MM-dd")

            df = {"name": name, "phone": phone, "date": date, "uid": uid}
            self.signaller.staff_list_add.emit(df)

            self.name_edit.clear()
            self.phone_edit.clear()
            self.date_edit.setDate(QDate.currentDate())
            return
        
        if self.mode == "delete":
            self.waiting_for_rfid = False

            # UID 비교
            if uid != self.delete_target_uid:
                QMessageBox.warning(self, "오류", "일치하지 않는 RFID입니다.")
                self.mode = None
                return

            # UID 일치 → 삭제 수행
            row = self.delete_target_row
            name = self.log_table.item(row, 0).text()

            self.log_table.removeRow(row)
            self.signaller.staff_delete_row.emit(row)

            QMessageBox.information(self, "삭제 완료", f"{name} 직원이 삭제되었습니다.")

            # 상태 초기화
            self.mode = None
            self.delete_target_row = None
            self.delete_target_uid = None


    def on_delete(self):
        self.pending_dialog = QMessageBox(self)
        msg = self.pending_dialog
        selected = self.log_table.selectedItems()

        if not selected:
            msg.warning(self, "오류", "삭제할 직원을 선택해주세요.")
            return

        # 첫 번째 선택된 행 기준
        row = selected[0].row()
        uid_item = self.log_table.item(row, 3)
        if not uid_item:
            msg.warning(self, "오류", "UID 정보를 찾을 수 없습니다.")
            return

        # 삭제 대상 저장
        self.delete_target_row = row
        self.delete_target_uid = uid_item.text()

        self.mode = "delete"
        self.waiting_for_rfid = True

        msg.setWindowTitle("RFID 확인")
        msg.setText("삭제할 직원의 RFID 카드를 인식해주세요.")
        msg.setStandardButtons(QMessageBox.StandardButton.Cancel)
        result = msg.exec()

        if result == QMessageBox.StandardButton.Cancel:
            self.waiting_for_rfid = False
            self.mode = None
            QMessageBox.information(self, "취소", "삭제가 취소되었습니다.")
            return


    def update_log_table(self, staff_list):
        # staff_list = [(name, phone, date, uid), ...]
        print("수신 완료")
        self.log_table.setRowCount(0)  # 기존 내용 지우기
        print(staff_list["phone"])
        for _, row in staff_list.iterrows():
            r = self.log_table.rowCount()
            self.log_table.insertRow(r)
            self.log_table.setItem(r, 0, QTableWidgetItem(row["name"]))
            self.log_table.setItem(r, 1, QTableWidgetItem(str(row["phone"])))
            self.log_table.setItem(r, 2, QTableWidgetItem(row["date"]))
            self.log_table.setItem(r, 3, QTableWidgetItem(row["uid"]))
            