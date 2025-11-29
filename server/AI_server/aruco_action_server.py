#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco 
import numpy as np
from PIL import ImageFont, ImageDraw, Image
from collections import deque 
import os
import socket 
import math
import time 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy # ⬅️ 추가 필수
from std_msgs.msg import Bool

# --- (헬퍼 함수 1: 한글 그리기) ---
FONT_PATH = "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf"
def draw_korean_text(frame, text, position, color_bgr=(0, 255, 0), font_size=15):
    img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)
    try:
        font = ImageFont.truetype(FONT_PATH, font_size)
    except IOError:
        font = ImageFont.load_default(size=font_size)
    color_rgb = (color_bgr[2], color_bgr[1], color_bgr[0])
    bbox = draw.textbbox(position, text, font=font)
    draw.rectangle(bbox, fill=(0,0,0,50)) 
    draw.text(position, text, font=font, fill=color_rgb)
    frame_with_text = cv2.cvtColor(np.array(img_pil), cv2.COLOR_BGR2RGB)
    return frame_with_text


class ArucoActionServer(Node): 
    def __init__(self):
        super().__init__('aruco_action_server')
        
        # --- 1. ArUco 모델 및 카메라 설정 ---
        self.get_logger().info("Loading ArUco marker detector...")
        
        # ArUco 딕셔너리 및 디텍터 설정 (4x4_50으로 변경하여 호환성 확보)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())
        
        # ⚠️ 카메라 보정 매개변수 (실제 값으로 대체해야 함)
        self.MARKER_SIZE = 0.04 # 마커 실제 크기 (4cm)
        self.K = np.array([
            [800.0, 0.0, 320.0],  
            [0.0, 800.0, 240.0],  
            [0.0, 0.0, 1.0]
        ])
        self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # --- 2. UDP 소켓 설정 ---
        self.PORT = 5000
        self.BUFFER_SIZE = 65536 
        try:
            # 모든 IP(0.0.0.0)에서 UDP 수신 대기
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('', self.PORT))
            self.sock.settimeout(1.0) # 1초 타임아웃
            self.get_logger().info(f"✅ UDP 서버 바인딩 완료 (Port: {self.PORT})")
        except Exception as e:
            self.get_logger().error(f"❌ UDP 포트 {self.PORT} 바인딩 오류: {e}")
            self.sock = None
            rclpy.shutdown()
            exit()


        # --- 3. 상태 및 설정값 ---
        self.FRAME_WIDTH = 640   
        self.FRAME_HEIGHT = 480  
        self.TARGET_MARKER_ID = 0
        self.EVASION_DISTANCE_THRESHOLD = 0.5 # 0.5m 이내 접근 시 회피

        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # --- 4. ROS 2 퍼블리셔 ---
        self.docking_pub = self.create_publisher(
            Bool, 
            '/trigger_docking', 
            qos_profile
        )

        self.docking_cooldown = False
        self.cooldown_timer = None
        self.get_logger().info("✅ /trigger_docking 퍼블리셔 생성됨.")

        # --- 5. 녹화 설정 ---
        RECORD_VIDEO = False # 녹화는 비활성화하고, 필요시 활성화
        TARGET_FPS = 20.0   
        self.writer = None 
        if RECORD_VIDEO:
            timestr = time.strftime("%Y%m%d_%H%M%S")
            output_filename = f'output_ARUCO_UDP_{timestr}.avi' 
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.writer = cv2.VideoWriter(output_filename, fourcc, TARGET_FPS, (self.FRAME_WIDTH, self.FRAME_HEIGHT))
            self.get_logger().info(f"[녹화 시작] 파일이 '{output_filename}'에 저장됩니다.")


    def reset_cooldown(self):
        # 쿨다운 해제 로직을 도킹 변수에 맞춰 수정
        self.get_logger().info("쿨다운 해제. 도킹 재시작 대기.")
        self.docking_cooldown = False # ⬅️ self.evasion_cooldown 대신 self.docking_cooldown 사용
        if self.cooldown_timer:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None

    def run_loop(self):
        """메인 실행 루프 (UDP 데이터 수신)"""
        while rclpy.ok():
            try:
                # --- A. UDP 데이터 수신 (블로킹) ---
                data, addr = self.sock.recvfrom(self.BUFFER_SIZE)
                
                # B. 데이터 디코딩
                np_arr = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is None:
                    self.get_logger().warn("수신 프레임 디코딩 실패", throttle_duration_sec=1.0)
                    continue
                
                # C. 프레임 크기 조정 및 전처리
                if frame.shape[1] != self.FRAME_WIDTH or frame.shape[0] != self.FRAME_HEIGHT:
                    frame = cv2.resize(frame, (self.FRAME_WIDTH, self.FRAME_HEIGHT))

                annotated_frame = frame.copy()
                gray = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2GRAY)

                # --- D. ArUco 마커 감지 및 자세 추정 ---
                corners, ids, rejected = self.detector.detectMarkers(gray)
                status_display_list = []
                is_threat_detected = False

                if ids is not None:
                    
                    # 자세 추정
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE, self.K, self.D)

                    # 감지된 마커들을 순회하며 처리
                    for i in range(len(ids)):
                        marker_id = ids[i][0]
                        tvec = tvecs[i][0]
                        
                        distance = np.linalg.norm(tvec)
                        
                        # VVVV [수정] 30cm 오프셋 보정 적용 VVVV
                        distance = distance - 0.30

                        # 3D 축 그리기
                        cv2.drawFrameAxes(annotated_frame, self.K, self.D, rvecs[i], tvecs[i], self.MARKER_SIZE * 0.5) 
                        aruco.drawDetectedMarkers(annotated_frame, corners)

                        # 📝 화면 표시 내용 구성
                        line1 = f"Marker ID: {marker_id}"
                        line2 = f"  └ Distance: {distance:.3f} m (Th: {self.EVASION_DISTANCE_THRESHOLD}m)"
                        status_display_list.append(line1); status_display_list.append(line2); status_display_list.append(" ") 

                        # --- E. ROS 2 회피 트리거 발행 ---
                        if marker_id == self.TARGET_MARKER_ID and distance < self.EVASION_DISTANCE_THRESHOLD:
                            is_threat_detected = True
                            
                            if not self.docking_cooldown:
                                self.get_logger().warn(f"Target ID {marker_id} ({distance:.3f}m)! docking 기동 요청 !!!")
                                self.docking_pub.publish(Bool(data=True))
                                self.docking_cooldown = True
                                # 5초 쿨다운 시작
                                self.cooldown_timer = self.create_timer(5.0, self.reset_cooldown) 
                
                # --- F. 텍스트 그리기 및 출력/녹화 ---
                y_pos = 30 
                for i, status_text in enumerate(status_display_list):
                    text_color = (0, 0, 255)
                    if is_threat_detected:
                         text_color = (0, 255, 255)
                    
                    annotated_frame = draw_korean_text(annotated_frame, status_text, (10, y_pos), text_color, font_size=20) 
                    y_pos += 25 
                    
                
                if self.writer is not None:
                    self.writer.write(annotated_frame)
                
                cv2.imshow("ArUco Marker Tracking (UDP Stream)", annotated_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # --- G. ROS 2 콜백 처리 (타이머 등을 작동시키기 위해 필수) ---
                rclpy.spin_once(self, timeout_sec=0.001)

            except socket.timeout:
                # 데이터가 안 들어오면 타임아웃 발생 (정상 대기)
                self.get_logger().info("UDP 수신 대기 중...", throttle_duration_sec=5.0)
                pass 
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"메인 루프 오류: {e}")
                break

        # --- 9. 종료 ---
        self.sock.close() 
        if self.writer is not None: 
            self.writer.release()   
            self.get_logger().info("녹화 파일 저장을 완료했습니다.")
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    aruco_server_node = ArucoActionServer()
    try:
        aruco_server_node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        aruco_server_node.get_logger().info("종료 및 리소스 정리...")
        aruco_server_node.destroy_node()

if __name__ == '__main__':
    main()