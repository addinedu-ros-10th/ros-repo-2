import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32 # Float32 추가
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
        
        # 🟢 [추가] Docking 상태 구독 (마스터 스위치 역할)
        self.create_subscription(Bool, '/docking_trigger', self.docking_status_callback, 10)
        
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
        
        # 🟢 [추가] Docking 상태 플래그
        self.is_docking_active = False 
        
        # [Final Fix] EVASION DURATION 및 속도/방향 변수 초기화
        self.EVASION_DURATION = 3.0     
        self.target_speed = 0.0
        self.turn_direction = 0.0       
        
        # [Final Fix] 20Hz로 연속적인 제어 명령을 보낼 타이머 (우선권 확보)
        self.control_timer = self.create_timer(0.05, self.control_loop) # 20Hz
        
        self.get_logger().info('✅ 회피 기동 제어 노드 시작됨.')

    # 🟢 [추가] Docking 상태 콜백 함수
    def docking_status_callback(self, msg: Bool):
        """AI Server 또는 Docking Controller에서 발행하는 Docking 상태를 수신"""
        was_active = self.is_docking_active
        self.is_docking_active = msg.data
        
        if self.is_docking_active and not was_active:
            # Docking 시작 시, 진행 중인 회피를 즉시 중단하고 제어권을 넘김
            if self.evasion_active:
                self.get_logger().warn('🚨 Docking 시작! 회피 기동 강제 종료 및 제어권 이양.')
                self.check_threat_timeout(force_stop=True) # 회피 타이머 및 상태 강제 종료

    def scan_callback(self, msg: LaserScan):
        self.lidar_data = msg
        
    def check_threat_timeout(self, force_stop=False):
        """회피 기동 종료 및 주행 재개 신호를 보냅니다."""
        
        # Docking이 활성화된 상태에서 시간 초과가 발생하면 (강제 종료가 아니면)
        # Docking Controller가 제어권을 유지해야 하므로 Nav2 재개 신호를 보내지 않습니다.
        if self.is_docking_active and not force_stop:
            self.get_logger().info('위협 해제. 그러나 Docking 중이므로 제어권 유지.')
            self.evasion_active = False
            return
            
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
        """evasion_active 상태일 때 20Hz로 연속적으로 회피 명령을 발행합니다."""
        
        # 🟢 [핵심 로직] Docking 중이거나 회피가 활성화되지 않으면, Nav2에게 제어권을 넘김 (Twist() 발행)
        if self.is_docking_active:
            # Docking Controller에게 제어권을 완전히 위임하기 위해 아무것도 발행하지 않거나 Twist() 발행 (여기는 아무것도 안 보냄)
            return

        elif self.evasion_active:
            # 회피가 활성화된 경우에만 명령을 발행하여 Nav2 명령을 덮어씀
            evade_cmd = Twist()
            evade_cmd.linear.x = -self.target_speed
            evade_cmd.angular.z = 0.5 * self.turn_direction
            self.cmd_vel_publisher.publish(evade_cmd)
        
        else:
            # 회피가 비활성화된 경우, 0을 발행하여 Nav2의 제어가 확실히 복구되도록 도움
            # Nav2가 제어권을 잡도록 보장하기 위해 Twist()를 명시적으로 보낼 필요는 없음 (Nav2가 자신의 명령을 보낼 것이기 때문)
            pass

    def evasion_callback(self, msg):
        if msg.data: # True가 수신되면 (위협 감지)
            
            # ❌ [삭제] 🟢 [수정된 로직] Docking 중에는 회피를 무시하는 로직을 제거했습니다.
            # ❌ if self.is_docking_active:
            # ❌     self.get_logger().info('📢 Docking 중이므로 회피 신호 무시.')
            # ❌     return 
            
            # 🟢 이제 위협 감지 시 무조건 회피 기동을 시작합니다.
            
            # --- 1. 후방 공간 확인 및 속도 결정 로직 (기존 코드 유지) ---
            if self.lidar_data is None:
                self.get_logger().warn('!!! LIDAR 데이터 미수신: 안전을 위해 정지 !!!')
                self.evasion_active = True 
                return

            # ... (후방 거리 계산 로직 및 속도 결정 로직 유지) ...
            
            # 4. 회피 명령 실행 (상태 업데이트 및 타이머 시작)
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