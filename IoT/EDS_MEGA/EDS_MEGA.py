import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)
print("✅ 연결됨. 3개의 RFID 및 4개의 가스 데이터 수신 중...\n")

last_status_time = time.time()

while True:
    # === 1초마다 상태 메시지 출력 ===
    now = time.time()
    if now - last_status_time >= 1.0:
        print("RFID 및 가스 데이터 수신 중...")
        last_status_time = now

    if ser.in_waiting >= 3:
        header = ser.read(1)
        if header == b'\xAA':
            dtype = ser.read(1)

            # === RFID 1 / 2 / 3 ===
            if dtype in [b'\x11', b'\x12', b'\x13']:
                uid_bytes = []
                while True:
                    b = ser.read(1)
                    if not b or b == b'\x55':
                        break
                    uid_bytes.append(b[0])
                if uid_bytes:
                    uid_str = ' '.join(f'{b:02X}' for b in uid_bytes)
                    reader_map = {b'\x11': "1번", b'\x12': "2번", b'\x13': "3번"}
                    reader = reader_map[dtype]
                    print(f"💳 RFID {reader} 카드 인식: {uid_str}")

            # === 가스 센서 데이터 ===
            elif dtype == b'\x20':
                data = ser.read(1)
                footer = ser.read(1)
                if footer == b'\x55':
                    packet = data[0]
                    active = []
                    if packet & (1 << 0): active.append("S1")
                    if packet & (1 << 1): active.append("S2")
                    if packet & (1 << 2): active.append("S3")
                    if packet & (1 << 3): active.append("S4")
                    if active:
                        print(f"🌫 가스 감지됨 → {' '.join(active)}")
