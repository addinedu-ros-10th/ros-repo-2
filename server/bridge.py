import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

import socket
import struct
import threading

from PyQt6.QtCore import QObject, pyqtSignal

# TCP 패킷 파싱용 상수
PACKET_SIZE = 16  # 1B id + 2B sensor + 12B xyz + 1B led


class ROSTCPBridge(Node, QObject):
    """
    ROS2 토픽(/robotXX/amcl_pose)을 구독해서
    GUI(Qt) 쪽으로 시그널을 보내고,
    동시에 TCP 서버도 돌리는 브리지 노드.
    """

    # (필요하면 직접 쓸 수 있는 Qt 시그널, 현재는 외부 signaller를 주로 사용)
    robot_signal = pyqtSignal(int, float, float)

    def __init__(self, signaller):
        # 다중 상속 초기화
        Node.__init__(self, 'ros_tcp_bridge')
        QObject.__init__(self)

        # GUI 쪽에서 만들어서 넘겨준 BridgeSignaller 인스턴스
        self.signaller = signaller

        # ---------------- ROS2 구독 설정 ----------------
        # /robot21/amcl_pose, /robot22/amcl_pose, /robot23/amcl_pose
        # 타입: geometry_msgs/msg/PoseWithCovarianceStamped
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

        # ---------------- TCP 서버 설정 ----------------
        self.host = "192.168.0.184"
        self.port = 2025
        self.stop_flag = False

        tcp_thread = threading.Thread(
            target=self.start_tcp_server,
            daemon=True
        )
        tcp_thread.start()

        print(f"[Bridge 시작] ROS2 수신 + TCP 수신 동시 실행 중 (포트 {self.port})")

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

    # ---------------- TCP 서버 ----------------

    def start_tcp_server(self):
        """
        외부 장치에서 보내는 TCP 패킷을 받아서 콘솔에 로그 출력.
        (현재는 ROS/GUI랑 직접 연결되진 않고 모니터링 용도)
        """
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
                        print(
                            f"[TCP 수신] ID={id_val}, 센서={sensor}, "
                            f"좌표=({x:.2f}, {y:.2f}, {z:.2f}), LED={led_text}"
                        )
                    except struct.error as e:
                        print(f"[패킷 파싱 오류] 구조체 언패킹 실패: {e}")

        except Exception as e:
            print(f"[오류] TCP 수신 중 예외 발생: {e}")
        finally:
            conn.close()
            server.close()
            print("[TCP 서버 종료]")
