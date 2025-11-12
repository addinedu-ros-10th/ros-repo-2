import paho.mqtt.client as mqtt
import struct

BROKER = "192.168.0.184"
TOPIC = "ai/object"

def start_mqtt():
    client = mqtt.Client()
    client.connect(BROKER, 1883, 60)
    print("[MQTT 연결 완료] AI 서버 → 통합 서버 데이터 전송 시작")

    # 예제용: 임의로 1초마다 데이터 전송
    try:
        while True:
            # 전송할 값 설정
            id_val = 1
            sensor = "OB"  # 2바이트 문자
            x, y, z = 1.23, 4.56, 7.89

            # 패킹 (1바이트 id + 2바이트 sensor + 12바이트 좌표)
            data = struct.pack('B2sfff', id_val, sensor.encode(), x, y, z)

            # MQTT 발행
            client.publish(TOPIC, data)
            print(f"[MQTT 발행] ID={id_val}, 센서={sensor}, 좌표=({x}, {y}, {z})")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[중단] MQTT 테스트 종료")
        client.disconnect()

if __name__ == "__main__":
    start_mqtt()