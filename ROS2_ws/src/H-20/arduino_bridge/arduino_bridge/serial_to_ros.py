import serial
import time
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String

class SerialToROS(Node):
    def __init__(self):
        super().__init__('serial_to_ros')
        # === 데이터 저장용 ===
        self.pos = []
        self.vel = []
        self.psd = []

        # ROS Publisher
        self.pub = self.create_publisher(String, 'arduino/data', 10)
        # === ROS2 Subscriber (PC -> Arduino) ===
        self.sub = self.create_subscription(String, 'pc_to_pi_cmd', self.command_callback, 10)
        # Serial
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        time.sleep(2)
        self.get_logger().info("Connected to Arduino Mega")

        # Timer = 100 Hz
        self.buzzer_sent = False
        self.timer = self.create_timer(0.01, self.read_serial)
        
        # 생성자에 추가
        self.check_connection_timer = self.create_timer(0.5, self.check_remote_connection)

    # Remote PC 연결 체크 함수
    def check_remote_connection(self):
        # pub.get_subscription_count() > 0이면 Remote PC가 구독 중
        if not self.buzzer_sent and self.pub.get_subscription_count() > 0:
            self.get_logger().info("Remote PC connected → sending buzzer command to Arduino")
            self.ser.write('B'.encode())  # 부저 ON
            self.buzzer_sent = True

    # --- Arduino → ROS2 ---
    # Arduino → ROS2 (JSON string으로 퍼블리시)
    def read_serial(self):
        line = self.ser.readline().decode().strip()
        if not line:
            return
        try:
            data = json.loads(line)  # Arduino에서 JSON 형태로 전송
        except json.JSONDecodeError:
            return

        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)
        self.get_logger().info(f"Published Arduino Data: {data}")
        
    # --- ROS2 → Arduino (명령 전송) ---
    def command_callback(self, msg: String):
        cmd = msg.data.strip()
        # ================================
        # Numpad Command Mapping (Direction)
        # --------------------------------
        #   q   w   e        ↖   ↑   ↗
        #   a   s   d   =>   ←   ■   →
        #   z   x   c        ↙   ↓   ↘
        #
        # Command Definitions:
        #  'w' → Forward (↑)
        #  'x' → Backward (↓)
        #  'a' → Left (←)
        #  'd' → Right (→)
        #  'q' → Forward-Left  (↖)
        #  'e' → Forward-Right (↗)
        #  'z' → Backward-Left (↙)
        #  'c' → Backward-Right(↘)
        #  's' → Stop (■)
        #  'r' → Left Turn
        #  'f' → Right Turn
        # ================================
        valid_cmds = {'q','w','e','a','s','d','z','x','c','r','f'}

        if cmd not in valid_cmds:
            self.get_logger().warn(f"Invalid command received: {cmd}")
            return

        # 아두이노로 시리얼 전송
        self.ser.write((cmd + "\n").encode())
        self.get_logger().info(f"Sent command to Arduino: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialToROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
