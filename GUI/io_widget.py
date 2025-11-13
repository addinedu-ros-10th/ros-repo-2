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
    QHeaderView, QComboBox, QHBoxLayout, QMessageBox, QDialog
)
from PyQt6.QtCore import pyqtSignal, QObject, QStringListModel
from PyQt6.QtGui import QStandardItemModel, QStandardItem

# --- (선택) BridgeSignaller에 시그널이 없다면 간단한 확장 예시 ---
# 실제 프로젝트의 BridgeSignaller이 이미 Qt 시그널을 제공하면 이 부분은 불필요합니다.
class IOSignaller(QObject):
    # payload: dict -> 서버로 전송
    io_send_signal = pyqtSignal(dict)
    # logs: list[dict] -> 서버에서 전달되는 전체 로그 혹은 변경 로그
    io_logs_signal = pyqtSignal(list)

# ------------------------- [입출고 위젯] -------------------------
class IOWidget(QWidget):
    """입/출고 입력 폼 + 로그 테이블"""

    def __init__(self, signaller: QObject):
        super().__init__()
        self.signaller = signaller

        # --- 입력 폼 생성 ---
        form_group = QGroupBox("입/출고 등록")
        form_layout = QFormLayout()

        self.product_combo = QComboBox()
        self.product_combo.setEditable(False)          # 사용자가 직접 입력도 가능하게 하려면 True
        self.product_combo.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.qty_spin = QSpinBox()
        self.qty_spin.setRange(0, 1000000)
        self.abnormal_check = QCheckBox("비정상(체크시 비정상)")
        self.confirmer_edit = QLineEdit()

        form_layout.addRow("제품명:", self.product_combo)

        if hasattr(self.signaller, "io_products_signal"):
            try:
                self.signaller.io_products_signal.connect(self.set_products)
            except Exception:
                pass

        # RFID 수신 연결(옵션): signaller에 rfid_signal이 있다면 연결
        self._rfid_dialog = None
        self._rfid_received = None
        if hasattr(self.signaller, "rfid_signal"):
            try:
                # rfid_signal should emit a single value (e.g., rfid id string/int)
                self.signaller.rfid_signal.connect(self._on_rfid_received)
            except Exception:
                pass

        # 버튼: 입고, 출고
        btn_layout = QHBoxLayout()
        self.btn_in = QPushButton("입고")
        self.btn_out = QPushButton("출고")
        self.btn_in.setFixedWidth(100)
        self.btn_out.setFixedWidth(100)
        btn_layout.addWidget(self.btn_in)
        btn_layout.addWidget(self.btn_out)

        # form_layout.addRow("제품명:", self.product_combo)
        form_layout.addRow("수량:", self.qty_spin)
        form_layout.addRow("", self.abnormal_check)
        form_layout.addRow("확인자:", self.confirmer_edit)
        form_layout.addRow("", btn_layout)
        form_group.setLayout(form_layout)

        # --- 로그 테이블 ---
        self.log_table = QTableWidget(0, 6)
        self.log_table.setHorizontalHeaderLabels(["제품명", "구분", "수량", "상태", "일시", "확인자"])
        header = self.log_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.log_table.setMinimumHeight(220)

        # 레이아웃 합치기
        layout = QVBoxLayout()
        layout.addWidget(form_group)
        layout.addWidget(self.log_table)
        self.setLayout(layout)

        # 연결
        self.btn_in.clicked.connect(lambda: self.on_submit("입고"))
        self.btn_out.clicked.connect(lambda: self.on_submit("출고"))

        # 로그 수신 연결: signaller이 io_logs_signal 또는 비슷한 시그널을 제공한다고 가정
        if hasattr(self.signaller, "io_logs_signal"):
            try:
                self.signaller.io_logs_signal.connect(self.update_logs)
            except Exception:
                pass

    def _on_rfid_received(self, rfid_value):
        """브리지에서 RFID를 받으면 호출됩니다. 다이얼로그가 열려있으면 수락 처리."""
        self._rfid_received = rfid_value
        if self._rfid_dialog is not None:
            # 다이얼로그가 열려있다면 accepted 처리
            try:
                self._rfid_dialog.accept()
            except Exception:
                pass

    def _prompt_rfid_dialog(self):
        """RFID 인식 요청 다이얼로그를 표시. 리턴: (accepted: bool)"""
        dlg = QDialog(self)
        dlg.setWindowTitle("확인자 인증")
        vbox = QVBoxLayout()
        label = QLabel("확인자 RFID를 인식해 주세요")
        vbox.addWidget(label)

        hbox = QHBoxLayout()
        btn_cancel = QPushButton("취소")
        btn_confirm = QPushButton("확인(스마트폰/수동)")
        hbox.addWidget(btn_confirm)
        hbox.addWidget(btn_cancel)
        vbox.addLayout(hbox)

        dlg.setLayout(vbox)

        # 버튼 동작: 취소는 reject, 확인은 accept
        btn_cancel.clicked.connect(dlg.reject)
        btn_confirm.clicked.connect(dlg.accept)

        # 저장해서 외부에서 accept() 호출 가능하도록 함
        self._rfid_dialog = dlg
        self._rfid_received = None

        result = dlg.exec()  # 모달로 대기: 반환값 QDialog.Accepted(1) 또는 Rejected(0)

        # 다이얼로그 종료 후 정리
        self._rfid_dialog = None
        accepted = (result == QDialog.DialogCode.Accepted)
        return accepted

    def on_submit(self, kind: str):
        """입고/출고 버튼 클릭 -> DB 스키마에 맞춘 payload 생성, 로컬 추가, 폼 초기화, 서버 전송"""
        product = self.product_combo.currentText().strip()
        qty = self.qty_spin.value()
        abnormal = self.abnormal_check.isChecked()
        confirmer_text = self.confirmer_edit.text().strip()
        if product == "선택하세요." or not product:
            return

        accepted = self._prompt_rfid_dialog()

        if not accepted:
            # 취소된 경우: 알림 보여주고 아무 작업도 하지 않음
            QMessageBox.information(self, "취소", "취소되었습니다.")
            return
        
        # 필요하면 확인자 정보에 RFID를 포함시키세요.
        confirmed_by_id = None
        if self._rfid_received is not None:
            confirmed_by_id = self._rfid_received
            # 선택적으로 확인자 텍스트에 RFID나 이름을 반영
            confirmer_text = confirmer_text or str(self._rfid_received)

        # DB 스키마에 맞춘 값 매핑
        if abnormal:
            inout_type = "return"
            status = "RETURNED"
        else:
            inout_type = "IN" if kind == "입고" else "OUT"
            status = "STORED"

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # confirmed_by는 서버에서 이름->id 매핑을 해주도록 문자열도 함께 보냄.
        # 서버가 이미 직원 ID를 알고 있다면 confirmed_by에 정수 ID를 채워 보내면 됩니다.
        payload = {
            "product_name": product,
            "inout_type": inout_type,      # 'IN' or 'OUT'
            "quantity": qty,
            "status": status,              # 'STORED' or 'RETURNED'
            "timestamp": timestamp,        # 'YYYY-MM-DD HH:MM:SS'
            # 두 필드를 모두 보냄: confirmed_by는 가능하면 int, 그렇지 않으면 confirmer_name을 보냄
            "confirmed_by": None,          # 서버가 id를 채워줄 경우 사용
            "confirmer_name": confirmer_text
        }

        # 1) 로컬 UI에 즉시 추가 (표시는 사람이 읽기 쉬운 형태 사용)
        display_entry = {
            "product_name": payload["product_name"],
            "inout_type": "입고" if inout_type == "IN" else ("출고" if inout_type == "OUT" else "반품"),
            "quantity": payload["quantity"],
            "status": "비정상" if status == "RETURNED" else "정상",
            "timestamp": payload["timestamp"],
            "confirmer": confirmer_text
        }
        self.append_log(display_entry)

        # 2) 폼 초기화
        self.product_combo.setCurrentIndex(0)
        self.qty_spin.setValue(0)
        self.abnormal_check.setChecked(False)
        self.confirmer_edit.clear()

        # 3) 서버 전송: signaller에 맞는 시그널/메서드 호출
        if hasattr(self.signaller, "io_send_signal"):
            try:
                self.signaller.io_send_signal.emit(payload)
            except Exception:
                pass
        if hasattr(self.signaller, "send_io"):
            try:
                self.signaller.send_io(payload)
            except Exception:
                pass

    def set_products(self, products: list):
        """콤보박스에 제품 목록 세팅 (products: list[str] or list[tuple(id, name)])"""
        if not isinstance(products, list):
            return
        self.product_combo.clear()
        # products가 (id, name) 튜플 리스트면 userData로 id 저장
        if products and isinstance(products[0], (list, tuple)) and len(products[0]) >= 2:
            for pid, name in products:
                self.product_combo.addItem(str(name), pid)
        else:
            for name in products:
                self.product_combo.addItem(str(name))

    def update_logs(self, logs: list):
        """
        서버/브리지가 DB에서 받아 보낸 로그 목록을 테이블에 채움.
        기대하는 각 항목의 키: product_name, inout_type, quantity, status, timestamp, confirmed_by (또는 confirmer_name)
        """
        if not isinstance(logs, list):
            return
        self.log_table.setRowCount(0)
        for entry in logs:
            # 서버가 confirmed_by(직원 id)만 보내면 서버가 confirmed_name도 함께 보내는 것이 좋음.
            confirmer_display = ""
            if entry.get("confirmed_name"):
                confirmer_display = entry.get("confirmed_name")
            elif entry.get("confirmed_by"):
                confirmer_display = str(entry.get("confirmed_by"))
            elif entry.get("confirmer_name"):
                confirmer_display = entry.get("confirmer_name")

            kind_display = "입고" if entry.get("inout_type") == "IN" else "출고"
            status_display = "비정상" if entry.get("status") == "RETURNED" else "정상"

            row = self.log_table.rowCount()
            self.log_table.insertRow(row)
            self.log_table.setItem(row, 0, QTableWidgetItem(str(entry.get("product_name", ""))))
            self.log_table.setItem(row, 1, QTableWidgetItem(kind_display))
            self.log_table.setItem(row, 2, QTableWidgetItem(str(entry.get("quantity", ""))))
            self.log_table.setItem(row, 3, QTableWidgetItem(status_display))
            self.log_table.setItem(row, 4, QTableWidgetItem(str(entry.get("timestamp", ""))))
            self.log_table.setItem(row, 5, QTableWidgetItem(confirmer_display))

    def append_log(self, entry: dict):
        """단일 로그 항목을 테이블에 추가 (entry: dict 형태)"""
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        self.log_table.setItem(row, 0, QTableWidgetItem(str(entry.get("product_name", ""))))
        self.log_table.setItem(row, 1, QTableWidgetItem(str(entry.get("inout_type", ""))))
        self.log_table.setItem(row, 2, QTableWidgetItem(str(entry.get("quantity", ""))))
        self.log_table.setItem(row, 3, QTableWidgetItem(str(entry.get("status", ""))))
        self.log_table.setItem(row, 4, QTableWidgetItem(str(entry.get("timestamp", ""))))
        self.log_table.setItem(row, 5, QTableWidgetItem(str(entry.get("confirmer", ""))))
        # 선택: 가장 아래로 스크롤
        self.log_table.scrollToItem(self.log_table.item(row, 0))