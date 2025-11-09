#cmd_vel_switcher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelSwitcher(Node):
    def __init__(self):
        super().__init__('cmd_vel_switcher')
        
        # WaypointRTRController에서 발행하는 제어 모드 구독
        self.create_subscription(String, '/control_mode', self.mode_callback, 10)
        
        # Nav2 (또는 수동) cmd_vel 구독
        self.create_subscription(Twist, '/cmd_vel', self.nav_callback, 10)
        
        # PID Controller cmd_vel 구독
        self.create_subscription(Twist, '/cmd_vel_pid', self.pid_callback, 10)
        
        # 로봇 베이스로 최종 명령 발행
        self.cmd_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        
        self.current_mode = "nav2"
        
        self.get_logger().info("🚀 CmdVelSwitcher 시작: WaypointRTRController의 모드에 따라 제어 토픽 중계")

    def mode_callback(self, msg):
        """WaypointRTRController 또는 PIDController에서 제어 모드 변경 수신"""
        new_mode = msg.data.lower()
        if self.current_mode != new_mode:
            self.current_mode = new_mode
            self.get_logger().info(f"🔄 제어 모드 변경: {self.current_mode}")

    def nav_callback(self, msg):
        """Nav2 명령 수신 및 모드가 'nav2'일 때 발행"""
        if self.current_mode == "nav2":
            self.cmd_pub.publish(msg)

    def pid_callback(self, msg):
        """PID 명령 수신 및 모드가 'pid'일 때 발행"""
        if self.current_mode == "pid":
            self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()