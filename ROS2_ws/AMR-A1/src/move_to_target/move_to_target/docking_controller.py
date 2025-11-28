#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time 
import numpy as np
import math
import tf_transformations # tf_transformations 모듈 사용

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # QoS Profile (안정적인 통신)
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # === Publishers ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pause_pub = self.create_publisher(Bool, '/pause_navigation', qos_profile)

        # === Subscribers ===
        self.create_subscription(Bool, '/docking_trigger', self.docking_trigger_callback, 10)        # 🚨 마커의 Pose를 Aruco 서버로부터 수신한다고 가정
        self.create_subscription(PoseStamped, '/aruco_target_pose', self.marker_pose_callback, 10) 

        # === 제어 파라미터 ===
        self.TARGET_DISTANCE = 0.10     # 최종 도킹 목표 거리 (10cm)
        self.P_ANGULAR = 1.5            # 회전 제어 게인 (P-Gain)
        self.P_LINEAR = 0.2             # 전진 제어 게인 (P-Gain)
        self.MAX_SPEED_ROT = 0.3        # 최대 회전 속도 (rad/s)
        self.MAX_SPEED_LIN = 0.1        # 최대 전진 속도 (m/s)

        # === 상태 변수 ===
        self.is_docking_active = False
        self.docking_start_time = 0.0
        self.docking_timer = None
        self.marker_pose = None         # 마커의 최신 Pose (Pose 객체)

        self.get_logger().info('✅ Docking Controller 시작됨. /trigger_docking 대기 중...')

    def marker_pose_callback(self, msg: PoseStamped):
        """Aruco 서버가 발행한 마커의 상대 Pose를 수신합니다."""
        # 이 Pose는 로봇 base_link 기준으로 마커의 위치를 나타내야 합니다.
        self.marker_pose = msg.pose
        
    def get_yaw_from_quaternion(self, q: Quaternion):
        """Quaternion 객체에서 Yaw 각도(라디안)를 추출합니다."""
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2] # Yaw 값

    def docking_trigger_callback(self, msg: Bool):
        if msg.data: # 도킹 시작 신호 (True)
            if not self.is_docking_active:
                self.get_logger().warn("🚀 도킹 시퀀스 시작! (마커 감지)")
                self.is_docking_active = True
                self.docking_start_time = time.time() 

                # Nav2 주행 중단 (제어권 확보)
                self.pause_pub.publish(Bool(data=True))

                # 20Hz로 정밀 제어 시작
                self.docking_timer = self.create_timer(0.05, self.docking_step)
        
        elif not msg.data and self.is_docking_active:
             self.get_logger().info("도킹 트리거 해제됨 → 도킹 취소")
             self.docking_finished()

    def docking_step(self):
        """도킹 시퀀스: 20Hz로 정밀 제어 명령을 보냅니다. (테스트용)"""
        # 🚨 이 로그가 20Hz로 찍혀야 타이머가 정상 작동하는 것입니다.
        self.get_logger().info("--- DOCKING TIMER FIRING ---", throttle_duration_sec=0.5) 
        
        if not self.is_docking_active:
            if self.docking_timer: self.docking_timer.cancel()
            return

        current_time = time.time()
        elapsed = current_time - self.docking_start_time
        
        # VVVV [테스트 로직] 2초간 단순 전진 VVVV
        if elapsed < 2.0:
             twist = Twist()
             twist.linear.x = 0.1 # 10cm/s로 전진
             self.cmd_vel_pub.publish(twist)
        else:
             self.docking_finished() # 2초 후 종료


        """
        도킹 시퀀스: 20Hz로 정밀 제어 명령을 보냅니다.
        [벽 수평 Align -> 전진] 반복 로직 수행.
        """
        #if not self.is_docking_active or self.marker_pose is None:
        #    return

        #marker_p = self.marker_pose.position
        #marker_q = self.marker_pose.orientation
        
        # 1. 종료 조건: 거리 체크
        # Aruco 라이브러리는 보통 tvec의 Z축을 거리(깊이)로 간주합니다.
        #distance_to_marker = marker_p.z 
        
        #if distance_to_marker < self.TARGET_DISTANCE:
        #    self.get_logger().info("✅ 최종 도킹 거리 (10cm) 달성! 정지.")
        #    self.docking_finished()
        #    return

        # 2. 각도 및 좌우 오차 계산
        
        # 🚨 마커의 Yaw는 벽과의 평행/수직을 나타내므로, 수직 제어를 위해 90도를 더해 로봇의 목표 각도를 구해야 합니다.
        # 이 로직은 복잡하므로, 가장 간단하게 마커의 X축(전진방향)이 로봇 정면에 있도록 하는 것으로 가정합니다.
        
        # 마커의 좌우 편차 (Y축): 마커가 로봇의 정면 중앙에서 얼마나 벗어났는가
        #angular_error = marker_p.y # Y축 값은 좌우 오차를 나타냄

        
        # 3. 제어 명령 생성 (Iterative Align and Advance)
        
        # 3A. Angular Control (정렬: 마커를 중앙에 맞추기)
        # 마커가 왼쪽(+Y)에 있으면 오른쪽(-Z)으로 회전해야 함
        #rotation_rate = -self.P_ANGULAR * angular_error 
        #rotation_rate = max(min(rotation_rate, self.MAX_SPEED_ROT), -self.MAX_SPEED_ROT) # 속도 제한

        # 3B. Linear Control (전진: 목표 거리까지 전진)
        # 거리가 멀수록 빠르게, 가까울수록 느리게
        #target_linear_speed = self.P_LINEAR * (distance_to_marker - self.TARGET_DISTANCE)
        #linear_speed = max(min(target_linear_speed, self.MAX_SPEED_LIN), 0.0) # 속도 제한 및 음수 방지

        
        #twist = Twist()
        #twist.angular.z = rotation_rate
        #twist.linear.x = linear_speed
        
        #self.cmd_vel_pub.publish(twist)
        
        # Log:
        #self.get_logger().info(f"Docking: Dist={distance_to_marker:.2f}m, Error={angular_error:.2f}, Vz={rotation_rate:.2f}")


    def docking_finished(self):
        """도킹 종료 시 모든 자원 해제 및 Nav2 재개."""
        if not self.is_docking_active:
            return

        self.get_logger().warn("🏁 도킹 완료 또는 오류 종료! 로봇 정지.")
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
        node.get_logger().info("Docking Controller 종료 중...")
    finally:
        node.docking_finished() # 안전하게 정지
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
