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

        match = self.staff_list[self.staff_list["uid"] == uid]
        if match.empty:
            QMessageBox.warning(self, "RFID 오류", "등록되지 않은 RFID입니다.")
            return

        staff_name = match.iloc[0]["name"]
        input_name = self._pending_confirmer

        target = self.staff_list[self.staff_list["name"] == input_name]
        if target.empty:
            QMessageBox.warning(self, "오류", "확인자 이름이 직원 목록에 없습니다.")
            return

        correct_uid = target.iloc[0]["uid"]

        if uid != correct_uid:
            QMessageBox.warning(self, "RFID 불일치", "확인자 RFID가 아닙니다.")
            return

        # RFID 성공 메시지
        QMessageBox.information(self, "인증 완료", f"{staff_name}님의 RFID가 인증되었습니다.")

        # 다이얼로그 accept 처리
        if self.pending_dialog:
            self.pending_dialog.accept()

        # --- ⭐ 여기서 최종 등록 작업 실행 ⭐ ---
        self.register_io_entry(staff_name)

    # ======================================================
    #                    주요 처리 로직
    # ======================================================

    def on_submit(self, kind: str):
        self._pending_kind = kind  # '입고' / '출고'
        self._pending_product = self.product_combo.currentText().strip()
        self._pending_qty = self.qty_spin.value()
        self._pending_abnormal = self.abnormal_check.isChecked()
        self._pending_confirmer = self.confirmer_edit.text().strip()

        if not self._pending_product or self._pending_product == "선택하세요.":
            QMessageBox.warning(self, "오류", "제품을 선택해 주세요.")
            return
        
        if not self._pending_confirmer:
            QMessageBox.warning(self, "오류", "확인자를 입력해 주세요.")
            return

        # RFID 인증 시작
        accepted = self._prompt_rfid_dialog()
        if not accepted:
            QMessageBox.information(self, "취소", "RFID 인증이 취소되었습니다.")
            return

        # 폼 초기화
        self.product_combo.setCurrentIndex(0)
        self.qty_spin.setValue(0)
        self.abnormal_check.setChecked(False)
        self.confirmer_edit.clear()

        # # 서버로 전송
        # if hasattr(self.signaller, "io_send_signal"):
        #     self.signaller.io_send_signal.emit(payload)
        # if hasattr(self.signaller, "send_io"):
        #     self.signaller.send_io(payload)

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

    def register_io_entry(self, confirmer_name):
        kind = self._pending_kind               # "입고" / "출고"
        product = self._pending_product
        qty = self._pending_qty
        abnormal = self._pending_abnormal       # True = 비정상

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # ===============================
        # ① 동작 규칙: 비정상 + 입고/출고 처리
        # ===============================
        if not abnormal:
            # 정상일 때
            if kind == "입고":
                inout_type = "IN"
                status = "STORED"
                display_inout = "입고"
                display_status = "정상"
            else:  # 출고
                inout_type = "OUT"
                status = "RELEASED"
                display_inout = "출고"
                display_status = "정상"
        else:
            # 비정상일 때
            if kind == "입고":
                # 반품 처리
                inout_type = "RETURN"
                status = "RETURNED"
                display_inout = "반품"
                display_status = "비정상"
            else:
                # 출고 불가 처리
                inout_type = "REJECT"
                status = "REJECTED"
                display_inout = "출고불가"
                display_status = "비정상"

        # ===============================
        # ② 화면 출력용 entry 구성
        # ===============================
        display_entry = {
            "product_name": product,
            "inout_type": display_inout,    # "입고" / "출고" / "반품" / "출고불가"
            "quantity": qty,
            "status": display_status,       # "정상" / "비정상"
            "timestamp": timestamp,
            "confirmer": confirmer_name
        }

        # 테이블 갱신
        self.append_log(display_entry)

        # ===============================
        # ③ 서버 전송용 payload 구성
        # ===============================
        payload = {
            "product_name": product,
            "inout_type": inout_type,       # "IN" / "OUT" / "RETURN" / "REJECT"
            "quantity": qty,
            "status": status,               # "STORED" / "RELEASED" / "RETURNED" / "REJECTED"
            "timestamp": timestamp,
            "confirmer_name": confirmer_name
        }

        # 송신 signal
        if display_entry["inout_type"] in ["입고", "출고"]:
            # print(f"[입출고 등록] 보냈음 {payload}")
            self.signaller.io_send_signal.emit(payload)

        # ===============================
        # ④ 입력 UI 초기화
        # ===============================
        self.product_combo.setCurrentIndex(0)
        self.qty_spin.setValue(0)
        self.abnormal_check.setChecked(False)
        self.confirmer_edit.clear()