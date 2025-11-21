import pandas as pd
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView, QPushButton, QFileDialog,
    QComboBox
)


STAFF_CSV_PATH = "./GUI/data/storage_list.csv"

ZONE_MAPPING = {
    "화장품": "1",
    "전자 부품": "2",
    "인형": "3",
    "공구": "4"
}

MAX_PER_SUBZONE = 2  # 한 세부 구역당 최대 수량

class StorageWidget(QWidget):
    """입출고 데이터를 관리하고 CSV로 저장하는 위젯"""

    def __init__(self, signaller):
        super().__init__()
        self.signaller = signaller

        # DataFrame 초기화
        try:
            self.df = pd.read_csv(STAFF_CSV_PATH, encoding="utf-8-sig")
        except FileNotFoundError:
            self.df = pd.DataFrame(columns=["제품명", "장소", "잔여 수량", "일시"])

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
        if entry["inout_type"] not in ["IN", "OUT"]:
            return

        print(f"[제품 현황 업데이트] {entry}")
        product_name = entry["product_name"]
        zone_num = ZONE_MAPPING.get(product_name, "미정")

        # 현재 같은 제품의 모든 row 가져오기
        existing_rows = self.df[self.df["제품명"] == product_name]

        # 출고 처리
        if entry["inout_type"] == "OUT":
            qty_to_remove = entry["quantity"]

            if existing_rows.empty:
                print("[출고 오류] 재고 없음.")
                return

            # 최신 구역 순으로 정렬
            sorted_rows = existing_rows.copy()
            sorted_rows["구역번호"] = sorted_rows["장소"].apply(
                lambda x: int(x.split("-")[1].replace("구역", ""))
            )
            sorted_rows = sorted_rows.sort_values(by="구역번호", ascending=False)

            # 역순으로 출고
            for idx, row in sorted_rows.iterrows():
                if qty_to_remove <= 0:
                    break

                current_qty = row["잔여 수량"]
                remove_here = min(current_qty, qty_to_remove)
                qty_to_remove -= remove_here

                # 수량 차감
                self.df.loc[idx, "잔여 수량"] = current_qty - remove_here

                # 0개면 row 삭제
                if self.df.loc[idx, "잔여 수량"] <= 0:
                    self.df = self.df.drop(idx)

            # 저장 & 테이블 갱신
            self.df = self.df.reset_index(drop=True)
            self.update_table_from_df()
            self.df.to_csv(STAFF_CSV_PATH, index=False, encoding="utf-8-sig")
            print("[출고 처리 완료]")
            return

        # 입고 처리
        if existing_rows.empty:
            # 첫 번째 구역 생성
            current_zone_index = 1
            current_qty = 0

            first_row = {
                "제품명": product_name,
                "장소": f"{zone_num}-1구역",
                "잔여 수량": 0,
                "일시": entry["timestamp"]
            }
            self.df = pd.concat([self.df, pd.DataFrame([first_row])], ignore_index=True)
        else:
            last_zone = existing_rows["장소"].iloc[-1]
            current_zone_index = int(last_zone.split("-")[1].replace("구역", ""))
            current_qty = existing_rows["잔여 수량"].iloc[-1]

        # 입고량 만큼 반복 처리
        for _ in range(entry["quantity"]):

            # 구역이 꽉 차면 다음 구역 생성
            if current_qty >= MAX_PER_SUBZONE:
                current_zone_index += 1
                current_qty = 0

                new_row = {
                    "제품명": product_name,
                    "장소": f"{zone_num}-{current_zone_index}구역",
                    "잔여 수량": 0,
                    "일시": entry["timestamp"]
                }
                self.df = pd.concat([self.df, pd.DataFrame([new_row])], ignore_index=True)

            # 해당 구역 수량 증가
            zone_label = f"{zone_num}-{current_zone_index}구역"
            row_mask = (self.df["제품명"] == product_name) & (self.df["장소"] == zone_label)
            self.df.loc[row_mask, "잔여 수량"] += 1
            current_qty += 1

        # 저장 & 갱신
        self.update_table_from_df()
        self.df.to_csv(STAFF_CSV_PATH, index=False, encoding="utf-8-sig")
        print("[입고 처리 완료]")

    # def save_csv(self):
    #     """CSV로 저장"""
    #     path, _ = QFileDialog.getSaveFileName(self, "CSV 저장", "", "CSV Files (*.csv)")
    #     if path:
    #         self.df.to_csv(path, index=False, encoding="utf-8-sig")