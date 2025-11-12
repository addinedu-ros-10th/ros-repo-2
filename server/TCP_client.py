import socket
import serial
import struct
import time

SERVER_IP = "192.168.0.184"  # ROS Bridge 서버 IP
PORT = 2025                  # Mosquitto와 충돌 피하기 위해 2025 사용

SERIAL_PORT = "/dev/ttyACM0" # 아두이노 메가 연결 포트 (리눅스 기준)
BAUDRATE = 115200

def start_serial_to_tcp():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"[시리얼 연결 완료] {SERIAL_PORT} ({BAUDRATE}bps)")

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((SERVER_IP, PORT))
    print(f"[TCP 연결 완료] {SERVER_IP}:{PORT}")

    try:
        while True:
            line = ser.readline().decode().strip()
            if not line:
                continue

            try:
                # 예: "1,OB,1.23,4.56,7.89,1"
                parts = line.split(',')
                id_val = int(parts[0])
                sensor = parts[1][:2]
                x, y, z = map(float, parts[2:5])
                led_state = int(parts[5]) if len(parts) > 5 else 0

                # 기존 데이터 패킹 (LED는 TCP로 안 보내고 print만)
                data = struct.pack('<B2sfffB', id_val, sensor.encode(), x, y, z, led_state)
                client.sendall(data)

                led_text = "ON" if led_state else "OFF"
                print(f"[TCP 송신] ID={id_val}, 센서={sensor}, 좌표=({x:.2f}, {y:.2f}, {z:.2f}), LED={led_text}")

            except Exception as e:
                print(f"[데이터 파싱 오류] '{line}' → {e}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[중단] 종료 중...")
    finally:
        ser.close()
        client.close()

if __name__ == "__main__":
    start_serial_to_tcp()