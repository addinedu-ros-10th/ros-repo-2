import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import socket
import struct
import threading
from PyQt6.QtCore import QObject, pyqtSignal
import pandas as pd

PACKET_SIZE = 16  # 1B id + 2B sensor + 12B xyz + 1B led


class ROSTCPBridge(Node, QObject):
    robot_signal = pyqtSignal(int, float, float)  # 클래스 속성으로 정의
    def __init__(self, signaller):
        Node.__init__(self, 'ros_tcp_bridge')   # ROS2 Node 초기화
        QObject.__init__(self)  

        self.signaller = signaller  # <-- 전달받은 signaller 저장

        # 직원 데이터프레임 초기화
        self.staff_list = pd.DataFrame(columns=["name", "phone", "date", "uid"])

        # staff 등록 신호 연결
        self.signaller.staff_list_add.connect(self.rfid_callback)
        print("서버: staff_list_add 시그널 연결 완료")

        self.create_subscription(Point, '/robot1/pos', self.robot1_callback, 10)
        self.create_subscription(Point, '/robot2/pos', self.robot2_callback, 10)
        self.create_subscription(Point, '/robot3/pos', self.robot3_callback, 10)

<<<<<<< HEAD
        self.host = "192.168.0.184"
=======
        self.host = "192.168.0.184"   # 서버 IP 192.168.2.7
>>>>>>> 299ee85e9f4487066448282979049bfcc5a89bc7
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
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 재사용 옵션
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"[TCP 서버 대기 중] {self.host}:{self.port}")

        while not self.stop_flag:   # 서버 전체 루프
            print("[TCP 연결 대기 중...]")
            conn, addr = server.accept()
            print(f"[TCP 연결 수락] 클라이언트: {addr}")

            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        print("[TCP 연결 종료 감지]")
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

                        except Exception as e:
                            print(f"[파싱 오류] {e}")

            except Exception as e:
                print(f"[TCP 오류] {e}")
            finally:
                conn.close()
                print("[TCP 연결 닫힘]")

        server.close()
        print("[TCP 서버 종료]")

    def rfid_callback(self, staff_data):
        print("서버: rfid_callback 호출됨:", staff_data)
        # # 직원 데이터프레임에 새 직원 추가
        # self.staff_list = pd.concat([self.staff_list, pd.DataFrame([staff_data])], ignore_index=True)
        # print("서버: 직원 등록 완료. 현재 직원 목록:\n", self.staff_list)

        # # GUI로 업데이트된 직원 목록 전송
        # self.signaller.staff_list_signal.emit(self.staff_list)  # DataFrame 전달

def main(signaller):
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
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

if __name__ == "__main__":
    main()
