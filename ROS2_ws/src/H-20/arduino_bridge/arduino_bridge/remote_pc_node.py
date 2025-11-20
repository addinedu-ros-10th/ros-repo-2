import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
from pynput import keyboard


class RemotePCNode(Node):
    def __init__(self):
        super().__init__('remote_pc_node')

        # --- ROS2 Subscribers ---
        self.sub = self.create_subscription(
            String,
            'arduino/data',
            self.arduino_callback,
            10
        )

        # --- ROS2 Publishers ---
        self.pub = self.create_publisher(String, 'pc_to_pi_cmd', 10)

        # --- Data Storage ---
        self.pos_data = []
        self.vel_data = []
        self.psd_data = []

        # --- Start Keyboard Listener in a separate thread ---
        threading.Thread(target=self.keyboard_listener, daemon=True).start()
        self.get_logger().info("Keyboard listener started.")

    # ================================================================
    # Arduino → PC 데이터 수신 콜백
    # ================================================================
    def arduino_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.pos_data = data.get("POS", [])
            self.vel_data = data.get("VEL", [])
            self.psd_data = data.get("PSD", [])

            self.get_logger().info(f"POS: {self.pos_data}")
            self.get_logger().info(f"VEL: {self.vel_data}")
            self.get_logger().info(f"PSD: {self.psd_data}")

        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON data received from Arduino")

    # ================================================================
    # 키보드 입력 → ROS2 Publish
    # ================================================================
    def keyboard_listener(self):
        def on_press(key):
            try:
                k = key.char  # q,w,e,a,s,d,z,x,c 등의 문자 입력만 처리
                if k in ['q','w','e','a','s','d','z','x','c','r','f']:
                    msg = String()
                    msg.data = k
                    self.pub.publish(msg)
                    #self.get_logger().info(f"[KEY] Command sent → {k}")
            except:
                pass

        # 비동기 키보드 리스너 시작
        listener = keyboard.Listener(on_press=on_press)
        listener.start()

    # ================================================================


def main(args=None):
    rclpy.init(args=args)
    node = RemotePCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
