import sys
import threading
from datetime import datetime
import pandas as pd
import os

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QStackedWidget,
    QTableWidget, QTableWidgetItem
)
from PyQt6.QtWidgets import (
    QLineEdit, QSpinBox, QCheckBox, QTextEdit, QGroupBox, QFormLayout,
    QHeaderView, QComboBox, QHBoxLayout, QMessageBox, QDialog
)
from PyQt6.QtCore import pyqtSignal, QObject, QStringListModel
from PyQt6.QtGui import QStandardItemModel, QStandardItem

STAFF_CSV_PATH = "./GUI/data/staff_list.csv"

# --- (선택) BridgeSignaller에 시그널이 없다면 간단한 확장 예시 ---
# 실제 프로젝트의 BridgeSignaller이 이미 Qt 시그널을 제공하면 이 부분은 불필요합니다.
class IOSignaller(QObject):
    # payload: dict -> 서버로 전송
    io_send_signal = pyqtSignal(dict)
    # logs: list[dict] -> 서버에서 전달되는 전체 로그 혹은 변경 로그
    io_logs_signal = pyqtSignal(list)

# ------------------------- [입출고 위젯] -------------------------
class IOWidget(QWidget):
    """입/출고 입력 폼 + RFID 확인 + 로그 테이블"""

    def __init__(self, signaller: QObject):
        super().__init__()
        self.signaller = signaller
        self.waiting_for_rfid = False
        self._rfid_dialog = None
        self._rfid_received = None
        self.pending_dialog = None
        self.log_entry = None

        # 직원 리스트 로드
        self.staff_list = pd.read_csv(STAFF_CSV_PATH)

        # -------------------- 입력 폼 --------------------
        form_group = QGroupBox("입/출고 등록")
        form_layout = QFormLayout()

        # 제품 선택
        self.product_combo = QComboBox()
        self.product_combo.setEditable(False)
        self.product_combo.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

        # 수량
        self.qty_spin = QSpinBox()
        self.qty_spin.setRange(0, 1000000)

        # 상태 체크
        self.abnormal_check = QCheckBox("비정상(체크시 비정상)")

        # 확인자 입력 (텍스트)
        self.confirmer_edit = QLineEdit()

        form_layout.addRow("제품명:", self.product_combo)
        form_layout.addRow("수량:", self.qty_spin)
        form_layout.addRow("", self.abnormal_check)
        form_layout.addRow("확인자:", self.confirmer_edit)

        # 버튼
        btn_layout = QHBoxLayout()
        self.btn_in = QPushButton("입고")
        self.btn_out = QPushButton("출고")
        btn_layout.addWidget(self.btn_in)
        btn_layout.addWidget(self.btn_out)
        form_layout.addRow("", btn_layout)

        form_group.setLayout(form_layout)

        # -------------------- 로그 테이블 --------------------
        self.log_table = QTableWidget(0, 6)
        self.log_table.setHorizontalHeaderLabels(
            ["제품명", "구분", "수량", "상태", "일시", "확인자"]
        )
        header = self.log_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.log_table.setMinimumHeight(220)

        # -------------------- 레이아웃 --------------------
        layout = QVBoxLayout()
        layout.addWidget(form_group)
        layout.addWidget(self.log_table)
        self.setLayout(layout)

        # -------------------- 시그널 연결 --------------------
        self.btn_in.clicked.connect(lambda: self.on_submit("입고"))
        self.btn_out.clicked.connect(lambda: self.on_submit("출고"))

        if hasattr(self.signaller, "io_products_signal"):
            self.signaller.io_products_signal.connect(self.set_products)

        if hasattr(self.signaller, "io_logs_signal"):
            self.signaller.io_logs_signal.connect(self.update_logs)

        # RFID 수신 시그널 연결
        if hasattr(self.signaller, "staff_rfid_signal_2"):
            self.signaller.staff_rfid_signal_2.connect(self.Receiving_confirmation_tag)

    # ======================================================
    #                RFID 인증 및 처리
    # ======================================================

    def _prompt_rfid_dialog(self):
        user = self.confirmer_edit.text().strip() or "확인자"

        dlg = QDialog(self)
        self.pending_dialog = dlg
        dlg.setWindowTitle("확인자 인증")

        v = QVBoxLayout()
        v.addWidget(QLabel(f'"{user}" 님의 RFID 카드를 태깅해 주세요'))

        btn_cancel = QPushButton("취소")
        btn_cancel.clicked.connect(dlg.reject)

        h = QHBoxLayout()
        h.addWidget(btn_cancel)
        v.addLayout(h)

        dlg.setLayout(v)

        self.waiting_for_rfid = True
        result = dlg.exec()

        self.waiting_for_rfid = False
        return result == QDialog.DialogCode.Accepted

    def Receiving_confirmation_tag(self, uid):
        if not self.waiting_for_rfid:
            return

        # 직원 리스트에서 UID 검색
        match = self.staff_list[self.staff_list["uid"] == uid]

        if match.empty:
            QMessageBox.warning(self, "RFID 오류", "등록되지 않은 RFID입니다.")
            return

        # 인식된 직원 정보
        staff_name = match.iloc[0]["name"]

        # 입력창의 확인자 이름
        input_name = self.confirmer_edit.text().strip()

        # 이름이 직원 리스트에 없으면 무조건 오류 처리
        target = self.staff_list[self.staff_list["name"] == input_name]
        if target.empty:
            QMessageBox.warning(self, "오류", "입력한 확인자 이름이 직원 목록에 없습니다.")
            return

        # UID가 입력한 이름과 연결된 직원인지 검사
        correct_uid = target.iloc[0]["uid"]

        if uid != correct_uid:
            QMessageBox.warning(
                self,
                "다른 직원 태그 감지",
                f"'{input_name}' 님의 RFID가 아닙니다.\n"
                f"다시 확인해 주세요."
            )
            return

        # 정상 인증 처리
        QMessageBox.information(self, "인증 완료", f"{staff_name}님의 RFID 인증이 확인되었습니다.")

        # 인증 성공이므로 다이얼로그를 accept 처리한다 (중요!!)
        if self.pending_dialog:
            self.pending_dialog.accept()

        self._rfid_received = uid

    # ======================================================
    #                    주요 처리 로직
    # ======================================================

    def on_submit(self, kind: str):
        product = self.product_combo.currentText().strip()
        qty = self.qty_spin.value()
        abnormal = self.abnormal_check.isChecked()
        confirmer_text = self.confirmer_edit.text().strip()

        if not product:
            return

        if not confirmer_text:
            QMessageBox.warning(self, "오류", "확인자 이름을 입력해주세요.")
            return

        # RFID 다이얼로그 호출
        accepted = self._prompt_rfid_dialog()
        if not accepted:
            QMessageBox.information(self, "취소", "취소되었습니다.")
            self.waiting_for_rfid = False
            return

        # RFID 처리
        confirmed_by = None
        if self._rfid_received is not None:
            confirmed_by = self._rfid_received
            confirmer_text = confirmer_text or confirmed_by

        # DB 매핑
        if abnormal:
            inout_type = "return"
            status = "RETURNED"
        else:
            inout_type = "IN" if kind == "입고" else "OUT"
            status = "STORED"

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        payload = {
            "product_name": product,
            "inout_type": inout_type,
            "quantity": qty,
            "status": status,
            "timestamp": timestamp,
            "confirmed_by": confirmed_by,
            "confirmer_name": confirmer_text,
        }

        # GUI 표시용
        display_entry = {
            "product_name": product,
            "inout_type": "입고" if inout_type == "IN" else ("출고" if inout_type == "OUT" else "반품"),
            "quantity": qty,
            "status": "비정상" if status == "RETURNED" else "정상",
            "timestamp": timestamp,
            "confirmer": confirmer_text,
        }

        self.log_entry = display_entry

        # 폼 초기화
        self.product_combo.setCurrentIndex(0)
        self.qty_spin.setValue(0)
        self.abnormal_check.setChecked(False)
        self.confirmer_edit.clear()

        # 서버로 전송
        if hasattr(self.signaller, "io_send_signal"):
            self.signaller.io_send_signal.emit(payload)
        if hasattr(self.signaller, "send_io"):
            self.signaller.send_io(payload)

    # ======================================================
    #                    부가 기능
    # ======================================================

    def set_products(self, products: list):
        self.product_combo.clear()

        if not products:
            return

        if isinstance(products[0], (list, tuple)) and len(products[0]) >= 2:
            for pid, name in products:
                self.product_combo.addItem(str(name), pid)
        else:
            for name in products:
                self.product_combo.addItem(str(name))

    def update_logs(self, logs: list):
        if not isinstance(logs, list):
            return

        self.log_table.setRowCount(0)
        for entry in logs:
            row = self.log_table.rowCount()
            self.log_table.insertRow(row)

            kind_display = "입고" if entry.get("inout_type") == "IN" else "출고"
            status_display = "비정상" if entry.get("status") == "RETURNED" else "정상"

            confirmer_display = (
                entry.get("confirmed_name")
                or entry.get("confirmer_name")
                or str(entry.get("confirmed_by", ""))
            )

            items = [
                entry.get("product_name", ""),
                kind_display,
                str(entry.get("quantity", "")),
                status_display,
                entry.get("timestamp", ""),
                confirmer_display,
            ]

            for col, text in enumerate(items):
                self.log_table.setItem(row, col, QTableWidgetItem(text))

    def append_log(self, entry: dict):
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        self.log_table.setItem(row, 0, QTableWidgetItem(str(entry.get("product_name", ""))))
        self.log_table.setItem(row, 1, QTableWidgetItem(str(entry.get("inout_type", ""))))
        self.log_table.setItem(row, 2, QTableWidgetItem(str(entry.get("quantity", ""))))
        self.log_table.setItem(row, 3, QTableWidgetItem(str(entry.get("status", ""))))
        self.log_table.setItem(row, 4, QTableWidgetItem(str(entry.get("timestamp", ""))))
        self.log_table.setItem(row, 5, QTableWidgetItem(str(entry.get("confirmer", ""))))
        self.log_table.scrollToItem(self.log_table.item(row, 0))
