#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np 
import math
import time

class EvasionController(Node):
    def __init__(self):
        super().__init__('evasion_controller')
        
        # 1. '회피' 신호 구독
        self.create_subscription(Bool, '/trigger_evasion', self.evasion_callback, 10)
        
        # 2. 'waypoint' 노드를 멈추게 할 토픽 발행
        self.pause_publisher = self.create_publisher(Bool, '/pause_navigation', 10)
            
        # 3. 로봇을 직접 제어(회피기동)하기 위한 /cmd_vel 발행
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 4. LIDAR 데이터 구독 (후방 안전거리 체크용)
        self.create_subscription(
            LaserScan,
            '/scan', 
            self.scan_callback,
            10
        )
        
        # === 상태 및 설정 ===
        self.lidar_data = None 
        self.evasion_active = False
        self.evasion_timer = None 
        
        # [Final Fix] EVASION DURATION 및 속도/방향 변수 초기화
        self.EVASION_DURATION = 3.0       # 3초 동안 회피 기동 유지
        self.target_speed = 0.0
        self.turn_direction = 0.0         # 1.0 (Left) or -1.0 (Right)
        
        # [Final Fix] 20Hz로 연속적인 제어 명령을 보낼 타이머 (우선권 확보)
        self.control_timer = self.create_timer(0.05, self.control_loop) # 20Hz
        
        self.get_logger().info('✅ 회피 기동 제어 노드 시작됨.')

    def scan_callback(self, msg: LaserScan):
        """최신 라이다 데이터를 저장합니다."""
        self.lidar_data = msg
        
    def check_threat_timeout(self):
        """회피 기동 종료 및 주행 재개 신호를 보냅니다."""
        self.get_logger().info('위협 해제: 일반 주행으로 복귀합니다.')
        self.evasion_active = False
        
        # 1. 정지 명령
        self.cmd_vel_publisher.publish(Twist())
        
        # 2. waypoint 노드 재개 신호 전송 (False)
        self.pause_publisher.publish(Bool(data=False))
        
        # 3. 타이머 삭제
        if self.evasion_timer:
            self.evasion_timer.cancel()
            self.evasion_timer = None
            
    def control_loop(self):
        """evasion_active 상태일 때 20Hz로 연속적으로 회피 명령을 발행하여 Nav2 명령을 덮어씁니다."""
        if self.evasion_active:
            evade_cmd = Twist()
            # evasion_callback에서 계산된 속도와 방향을 사용
            evade_cmd.linear.x = -self.target_speed
            evade_cmd.angular.z = 0.5 * self.turn_direction
            self.cmd_vel_publisher.publish(evade_cmd)
        
        else:
             # 정지 상태일 때는 0을 발행하여 Nav2의 제어가 확실히 복구되도록 도움
             self.cmd_vel_publisher.publish(Twist())


    def evasion_callback(self, msg):
        if msg.data: # True가 수신되면 (위협 감지)
            
            # --- 1. 후방 공간 확인 및 속도 결정 로직 ---
            if self.lidar_data is None:
                self.get_logger().warn('!!! LIDAR 데이터 미수신: 안전을 위해 정지 !!!')
                self.evasion_active = True # 상태만 켜서 control_loop에서 정지
                return

            ranges = np.array(self.lidar_data.ranges)
            num_ranges = len(ranges)
            
            # (후방 거리 계산 - min_rear_distance 결정 로직)
            center_index = num_ranges // 2
            half_check = num_ranges // 8
            rear_indices = list(range(center_index - half_check, center_index + half_check))
            rear_distances = [ranges[i] for i in rear_indices if ranges[i] > 0.01 and ranges[i] < float('inf')]
            min_rear_distance = min(rear_distances) if rear_distances else 5.0

            # --- 2. 속도/방향 결정 ---
            SAFE_BACKUP_DISTANCE = 0.50
            MAX_BACKUP_SPEED = 0.3 
            
            if min_rear_distance < SAFE_BACKUP_DISTANCE:
                 self.get_logger().warn(f'!!! 후방 장애물 ({min_rear_distance:.2f}m) 너무 가까움. 정지!!!')
                 self.evasion_active = True # control_loop에서 정지 Twist() 발행
                 return # 후진 대신 정지 상태 유지
            
            # 안전 속도 계산
            self.target_speed = min(MAX_BACKUP_SPEED, max(0.0, (min_rear_distance - SAFE_BACKUP_DISTANCE) * 0.8))


            # --- 3. 회전 방향 결정 (가장 넓은 공간 찾기) ---
            points_per_side = num_ranges // 8
            right_arc = ranges[:points_per_side] 
            left_arc = ranges[num_ranges - points_per_side:]
            
            avg_dist_right = np.mean([d for d in right_arc if d > 0.01 and d < float('inf')])
            avg_dist_left = np.mean([d for d in left_arc if d > 0.01 and d < float('inf')])
            
            self.turn_direction = 1.0 if avg_dist_left > avg_dist_right else -1.0 
            self.get_logger().info(f'🧭 Decided: Turning {"LEFT" if self.turn_direction > 0 else "RIGHT"} (Clearer space)')
            
            
            # --- 4. 회피 명령 실행 (상태 업데이트 및 타이머 시작) ---
            if not self.evasion_active:
                self.get_logger().warn('!!! 위협 감지: 회피 기동 시작 !!!')
                self.evasion_active = True
                self.pause_publisher.publish(Bool(data=True)) # Nav2 멈춤 신호

            # 5. 타이머 리셋
            if self.evasion_timer is not None:
                self.evasion_timer.cancel()
                
            self.evasion_timer = self.create_timer(self.EVASION_DURATION, self.check_threat_timeout)


def main(args=None):
    rclpy.init(args=args)
    node = EvasionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
