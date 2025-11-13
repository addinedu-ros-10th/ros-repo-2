import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import socket
import struct
import threading
from PyQt6.QtCore import QObject, pyqtSignal

PACKET_SIZE = 16  # 1B id + 2B sensor + 12B xyz + 1B led

class ROSTCPBridge(Node, QObject):
    robot_signal = pyqtSignal(int, float, float)  # 클래스 속성으로 정의
    def __init__(self, signaller):
        Node.__init__(self, 'ros_tcp_bridge')   # ROS2 Node 초기화
        QObject.__init__(self)                  # Qt QObject 초기화

        self.signaller = signaller  # <-- 전달받은 signaller 저장

        self.create_subscription(Point, '/robot1/pos', self.robot1_callback, 10)
        self.create_subscription(Point, '/robot2/pos', self.robot2_callback, 10)
        self.create_subscription(Point, '/robot3/pos', self.robot3_callback, 10)

        self.host = "192.168.0.184"
        self.port = 2025
        self.stop_flag = False

        tcp_thread = threading.Thread(target=self.start_tcp_server, daemon=True)
        tcp_thread.start()

        print(f"[Bridge 시작] ROS2 수신 + TCP 수신 동시 실행 중 (포트 {self.port})")

    def robot1_callback(self, msg):
        domain_id = 21
        print(f"[ROS2 수신] 로봇1 위치: ({msg.x:.2f}, {msg.y:.2f})")
        self.signaller.robot_signal.emit(domain_id, msg.x, msg.y)  # 수정

    def robot2_callback(self, msg):
        domain_id = 22
        print(f"[ROS2 수신] 로봇2 위치: ({msg.x:.2f}, {msg.y:.2f})")
        self.signaller.robot_signal.emit(domain_id, msg.x, msg.y)  # 수정

    def robot3_callback(self, msg):
        domain_id = 23
        print(f"[ROS2 수신] 로봇3 위치: ({msg.x:.2f}, {msg.y:.2f})")
        self.signaller.robot_signal.emit(domain_id, msg.x, msg.y)  # 수정

    def start_tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"[TCP 서버 대기 중] {self.host}:{self.port}")

        conn, addr = server.accept()
        print(f"[TCP 연결 수락] 클라이언트: {addr}")

        buffer = b""
        try:
            while not self.stop_flag:
                chunk = conn.recv(1024)
                if not chunk:
                    print("[TCP 연결 종료 감지]")
                    break

                buffer += chunk

                # 정확히 PACKET_SIZE 단위로 처리
                while len(buffer) >= PACKET_SIZE:
                    packet = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]

                    try:
                        id_val = packet[0]
                        sensor = packet[1:3].decode(errors='ignore')
                        x, y, z, led_state = struct.unpack('<fffB', packet[3:16])
                        led_text = "ON" if led_state else "OFF"
                        print(f"[TCP 수신] ID={id_val}, 센서={sensor}, 좌표=({x:.2f}, {y:.2f}, {z:.2f}), LED={led_text}")
                    except struct.error as e:
                        print(f"[패킷 파싱 오류] 구조체 언패킹 실패: {e}")

        except Exception as e:
            print(f"[오류] TCP 수신 중 예외 발생: {e}")
        finally:
            conn.close()
            server.close()
            print("[TCP 서버 종료]")

def main():
    rclpy.init()
    node = ROSTCPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_flag = True
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
