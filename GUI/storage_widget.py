import pandas as pd
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView, QPushButton, QFileDialog,
    QComboBox
)


STAFF_CSV_PATH = "./GUI/data/staff_list.csv"

ZONE_MAPPING = {
    "화장품": "1구역",
    "전자 부품": "2구역",
    "인형": "3구역",
    "공구": "4구역"
}

class StorageWidget(QWidget):
    """입출고 데이터를 관리하고 CSV로 저장하는 위젯"""

    def __init__(self, signaller):
        super().__init__()
        self.signaller = signaller

        # DataFrame 초기화
        try:
            self.df = pd.read_csv(self.csv_path, encoding="utf-8-sig")
        except FileNotFoundError:
            self.df = pd.DataFrame(columns=["제품명", "장소", "수량", "일시"])

                # 제품 선택
        self.product_combo = QComboBox()
        self.product_combo.setEditable(False)
        self.product_combo.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

        # 테이블 생성
        self.table = QTableWidget(0, len(self.df.columns))
        self.table.setHorizontalHeaderLabels(self.df.columns.tolist())
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

        # CSV 내용을 테이블에 반영
        self.update_table_from_df()

        # 입출고 signaller data 수신 시 업데이트
        # self.signaller.io_send_signal.connect(self.update_storage)  # signaller는 main

        # CSV 저장 버튼
        # self.save_btn = QPushButton("CSV 저장")
        # self.save_btn.clicked.connect(self.save_csv)

        layout = QVBoxLayout()
        layout.addWidget(self.product_combo)
        layout.addWidget(self.table)
        # layout.addWidget(self.save_btn)
        self.setLayout(layout)

    def update_table_from_df(self):
        """DataFrame 내용을 테이블에 반영"""
        self.table.setRowCount(0)
        for _, row in self.df.iterrows():
            row_idx = self.table.rowCount()
            self.table.insertRow(row_idx)
            for col_idx, col in enumerate(self.df.columns):
                self.table.setItem(row_idx, col_idx, QTableWidgetItem(str(row[col])))

    def update_storage(self, entry: dict):
        
        """입출고 데이터를 DataFrame과 테이블에 추가"""
        # 비정상 데이터는 저장하지 않음
        if entry["inout_type"] not in ["IN", "OUT"]:
            return

        print(f"[제품 현황 업데이트] {entry}")
        product_name = entry["product_name"]
        zone = ZONE_MAPPING.get(product_name, "미정")

        row_data = {
            "제품명": product_name,
            "장소": zone,
            "수량": entry["quantity"],
            "일시": entry["timestamp"]
        }

        # DataFrame에 추가
        self.df = pd.concat([self.df, pd.DataFrame([row_data])], ignore_index=True)

        # 테이블에도 반영
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, key in enumerate(["제품명", "장소", "수량", "일시"]):  # df.columns 대신 명시
            self.table.setItem(row, col, QTableWidgetItem(str(row_data.get(key, ""))))

        # CSV로 자동 저장
        self.df.to_csv(STAFF_CSV_PATH, index=False, encoding="utf-8-sig")
        print(f"보관현황 CSV 저장 완료: {STAFF_CSV_PATH}")

    def save_csv(self):
        """CSV로 저장"""
        path, _ = QFileDialog.getSaveFileName(self, "CSV 저장", "", "CSV Files (*.csv)")
        if path:
            self.df.to_csv(path, index=False, encoding="utf-8-sig")