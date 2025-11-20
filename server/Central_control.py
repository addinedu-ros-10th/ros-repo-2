import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import socket
import struct
import threading
from PyQt6.QtCore import QObject, pyqtSignal
import pandas as pd
from geometry_msgs.msg import PoseWithCovarianceStamped

PACKET_SIZE = 16  # 1B id + 2B sensor + 12B xyz + 1B led
HOST = "192.168.2.7"      # 서버 IP 192.168.2.7
PORT_ARDUINO = 2025     # Arduino TCP 서버 포트
PORT_AI = 2222         # AI TCP 서버 포트
STOP_FLAG = False

class ROSTCPBridge(Node, QObject):
    robot_signal = pyqtSignal(int, float, float)  # 클래스 속성으로 정의
    def __init__(self, signaller):
        Node.__init__(self, 'ros_tcp_bridge')   # ROS2 Node 초기화
        QObject.__init__(self)

        self.signaller = signaller  # <-- 전달받은 signaller 저장

        # 직원 데이터프레임 초기화
        self.staff_list = pd.DataFrame(columns=["name", "phone", "date", "uid"])

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot21/amcl_pose',
            self.robot1_callback,
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot22/amcl_pose',
            self.robot2_callback,
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot23/amcl_pose',
            self.robot3_callback,
            10
        )

        tcp_thread = threading.Thread(target=self.Arduino_tcp_server, daemon=True)
        tcp_thread.start()

        print(f"[Bridge 시작] ROS2 수신 + TCP 수신 동시 실행 중 (포트 {PORT_ARDUINO})")

    # ---------------- ROS 콜백 ----------------

    def _emit_robot(self, domain_id: int, msg: PoseWithCovarianceStamped):
        """
        공통 로직: PoseWithCovarianceStamped에서 x, y를 뽑아 GUI로 전달
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        print(f"[ROS2 수신] 로봇{domain_id} AMCL 위치: ({x:.2f}, {y:.2f})")

        # GUI 쪽 BridgeSignaller로 전달
        # (MainWindow에서 signaller.robot_signal.connect(update_gui) 로 연결되어 있음)
        self.signaller.robot_signal.emit(domain_id, x, y)

    def robot1_callback(self, msg: PoseWithCovarianceStamped):
        self._emit_robot(domain_id=21, msg=msg)

    def robot2_callback(self, msg: PoseWithCovarianceStamped):
        self._emit_robot(domain_id=22, msg=msg)

    def robot3_callback(self, msg: PoseWithCovarianceStamped):
        self._emit_robot(domain_id=23, msg=msg)

    def Arduino_tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 재사용 옵션
        server.bind((HOST,  PORT_ARDUINO))
        server.listen(1)
        print(f"[Arduion TCP 서버 대기 중] {HOST}:{PORT_ARDUINO}")

        while not STOP_FLAG:   # 서버 전체 루프
            print("[Arduion 연결 대기 중...]")
            conn, addr = server.accept()
            print(f"[Arduion 연결 수락] 클라이언트: {addr}")

            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        print("[Arduion 연결 종료 감지]")
                        break

                    msg = data.decode()

                    if msg.startswith("RFID"):
                        try:
                            # "RFID 3번 → 16 FC 40 02" → ["RFID 3번", "16 FC 40 02"]
                            parts = msg.split("→")
                            reader_info = parts[0].strip()   # "RFID 3번"
                            print("📩 수신:", reader_info)
                            uid_str = parts[1].strip()       # "16 FC 40 02"

                            # 원하는 리더기 번호만 필터링 (예: "RFID 1번")
                            if reader_info == "RFID 1번":
                                self.signaller.staff_rfid_signal_1.emit(uid_str)
                                self.signaller.staff_rfid_signal_2.emit(uid_str)
                                self.signaller.staff_rfid_signal_3.emit(uid_str)

                            if reader_info == "RFID 2번":
                                self.signaller.staff_rfid_signal_2.emit(uid_str)

                            if reader_info == "RFID 3번":
                                self.signaller.staff_rfid_signal_3.emit(uid_str)

                        except Exception as e:
                            print(f"[파싱 오류] {e}")

            except Exception as e:
                print(f"[Arduion 연결 오류] {e}")
            finally:
                conn.close()
                print("[Arduion 연결 닫힘]")

        server.close()
        print("[Arduion 연결 종료]")

class AIServerTCP:
    def __init__(self, host= HOST, port=PORT_AI):
        self.host = host
        self.port = port
        self.stop_flag = False

    def start_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"[AI TCP 서버 대기 중] {self.host}:{self.port}")

        while not self.stop_flag:
            print("[AI TCP 연결 대기 중...]")
            conn, addr = server.accept()
            print(f"[AI TCP 연결 수락] 클라이언트: {addr}")

            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        print("[AI TCP 연결 종료 감지]")
                        break

                    msg = data.decode().strip()
                    print(f"[AI 서버 메시지 수신] {msg}")

                    # 여기서 원하는 처리 추가 가능
                    # 예: 특정 키워드에 따라 다른 동작 수행
                    if msg.startswith("CMD"):
                        print(f"→ 명령어 처리: {msg}")
                    elif msg.startswith("LOG"):
                        print(f"→ 로그 메시지: {msg}")
                    else:
                        print(f"→ 일반 메시지: {msg}")

            except Exception as e:
                print(f"[AI TCP 오류] {e}")
            finally:
                conn.close()
                print("[AI TCP 연결 닫힘]")

        server.close()
        print("[AI TCP 서버 종료]")

class DummySignaller(QObject):
    robot_signal = pyqtSignal(int, float, float)
    staff_rfid_signal_1 = pyqtSignal(str)
    staff_rfid_signal_2 = pyqtSignal(str)
    staff_rfid_signal_3 = pyqtSignal(str)

def main(signaller):

    # 더미 Signaller 생성
    signaller = DummySignaller()

    rclpy.init()
    node = ROSTCPBridge(signaller)

    def ros_spin():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.stop_flag = True
            node.destroy_node()
            rclpy.shutdown()

    # ROS2 스레드 실행
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    # AI TCP 서버 스레드
    ai_server = AIServerTCP()
    ai_thread = threading.Thread(target=ai_server.start_server)
    ai_thread.start()

    # 메인 스레드에서 두 스레드가 종료될 때까지 대기
    try:
        ros_thread.join()
        ai_thread.join()
    except KeyboardInterrupt:
        print("[메인 종료]")

if __name__ == "__main__":
    main()
