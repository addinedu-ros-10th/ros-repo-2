#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time 
import numpy as np
import math
import tf_transformations

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # QoS Profile (안정적인 통신)
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # === Publishers ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pause_pub = self.create_publisher(Bool, '/pause_navigation', qos_profile)

        # === Subscribers ===
        self.create_subscription(Bool, '/docking_trigger', self.docking_trigger_callback, 10)
        self.create_subscription(PoseStamped, '/aruco_target_pose', self.marker_pose_callback, 10) 
        
        # 🟢 [추가] Evasion 신호를 구독하여 Docking 중단 로직 실행
        self.create_subscription(Bool, '/trigger_evasion', self.evasion_interrupt_callback, 10) 

        # === 제어 파라미터 ===
        self.TARGET_DISTANCE = 0.10     
        self.P_ANGULAR = 1.5            
        self.P_LINEAR = 0.2             
        self.MAX_SPEED_ROT = 0.3        
        self.MAX_SPEED_LIN = 0.1        

        # === 상태 변수 ===
        self.is_docking_active = False
        self.docking_start_time = 0.0
        self.docking_timer = None
        self.marker_pose = None         

        self.get_logger().info('✅ Docking Controller 시작됨. Evasion 인터럽트 준비 완료.')


    # 🟢 [추가] Evasion 인터럽트 콜백 함수 (우선순위 확보)
    def evasion_interrupt_callback(self, msg: Bool):
        """Evasion 신호 수신 시 Docking을 즉시 중단하고 제어권을 양보합니다."""
        if msg.data and self.is_docking_active:
            self.get_logger().warn("🛑 Evasion Interrupt: Docking 강제 중단 및 제어권 양보!")
            self.docking_finished() # DockingController의 제어권 즉시 포기 로직 실행

    def marker_pose_callback(self, msg: PoseStamped):
        self.marker_pose = msg.pose
        # (생략: Pose 추출 로직)

    def get_yaw_from_quaternion(self, q: Quaternion):
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

    def docking_trigger_callback(self, msg: Bool):
        if msg.data: # 도킹 시작 신호 (True)
            if not self.is_docking_active:
                self.get_logger().warn("🚀 도킹 시퀀스 시작!")
                self.is_docking_active = True
                self.docking_start_time = time.time() 

                self.pause_pub.publish(Bool(data=True))

                self.docking_timer = self.create_timer(0.05, self.docking_step)
        
        elif not msg.data and self.is_docking_active:
             self.get_logger().info("도킹 트리거 해제 → 도킹 취소")
             self.docking_finished()

    def docking_step(self):
        """도킹 시퀀스: 20Hz로 정밀 제어 명령을 보냅니다."""
        # 🚨 이 로직은 주석 처리된 상태를 해제하고 Aruco 제어를 구현해야 합니다.
        if not self.is_docking_active:
             if self.docking_timer: self.docking_timer.cancel()
             return
             
        # VVVV [테스트 로직] 2초간 단순 전진 VVVV
        current_time = time.time()
        elapsed = current_time - self.docking_start_time
        if elapsed < 2.0:
             twist = Twist()
             twist.linear.x = 0.1 
             self.cmd_vel_pub.publish(twist)
        else:
             self.docking_finished() # 2초 후 종료
        # ^^^^ [테스트 로직] ^^^^


    def docking_finished(self):
        """도킹 종료 시 모든 자원 해제 및 Nav2 재개."""
        if not self.is_docking_active:
            return

        self.get_logger().warn("🏁 도킹 종료 (정상 완료 또는 중단)!")
        self.is_docking_active = False

        # 1. 정지 명령
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        # 2. Nav2 재개
        self.pause_pub.publish(Bool(data=False))

        # 3. 타이머 정리
        if self.docking_timer is not None:
            self.docking_timer.cancel()
            self.docking_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()