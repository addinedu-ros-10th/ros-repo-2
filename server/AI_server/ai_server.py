#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import numpy as np
from PIL import ImageFont, ImageDraw, Image
from ultralytics.utils.plotting import Annotator, colors 
from collections import deque 
import os
import socket 
import math
import time 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2.aruco as aruco 

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

# --- (헬퍼 함수 2: BBox 기반 정규화 계수 계산) ---
def get_normalization_factor(bbox):
    x1, y1, x2, y2 = bbox
    height = y2 - y1
    return max(height, 50.0)

# 🟢 [수정된 헬퍼 함수] F_STD_Sum 기반 포즈 분석 (Max -> Sum)
def analyze_pose_by_f_std_sum(track_id, kpts, bbox, history_dict, conf_thresh, window_size, dt_run_threshold, dt_walk_threshold):
    status = "초기화 중..."
    debug_norm_sum = 0.0
    
    if track_id not in history_dict:
        history_dict[track_id] = {'left_y': deque(maxlen=window_size), 'right_y': deque(maxlen=window_size)}
        
    l_ankle = kpts[15].tolist(); r_ankle = kpts[16].tolist()
    l_y = int(l_ankle[1]) if l_ankle[2] > conf_thresh else -1
    r_y = int(r_ankle[1]) if r_ankle[2] > conf_thresh else -1
    
    norm_factor = get_normalization_factor(bbox)
    
    # 1. 히스토리 업데이트 및 STD 계산
    norm_std_left = 0.0 # 초기화
    norm_std_right = 0.0 # 초기화
    
    # 왼쪽 발목 처리
    if l_y != -1: 
        history_dict[track_id]['left_y'].append(l_y)
        if len(history_dict[track_id]['left_y']) == window_size:
            std_l = np.std(np.array(history_dict[track_id]['left_y']))
            norm_std_left = std_l / norm_factor
    else: 
        history_dict[track_id]['left_y'].clear()
        
    # 오른쪽 발목 처리
    if r_y != -1: 
        history_dict[track_id]['right_y'].append(r_y)
        if len(history_dict[track_id]['right_y']) == window_size:
            std_r = np.std(np.array(history_dict[track_id]['right_y']))
            norm_std_right = std_r / norm_factor
    else: 
        history_dict[track_id]['right_y'].clear()

    # 2. 분류 결정
    if (len(history_dict[track_id]['left_y']) == window_size and 
        len(history_dict[track_id]['right_y']) == window_size):

        # 🟢 [핵심 로직] 두 발목 STD의 합계 사용 (Sum)
        f_std_sum = norm_std_left + norm_std_right
        debug_norm_sum = f_std_sum
        
        # 🟢 [새로운 임계값] 0.02, 0.06 적용
        if f_std_sum < dt_walk_threshold: # < 0.02 (정지)
            status = "정지 (Stoping)"
        elif f_std_sum < dt_run_threshold: # 0.02 <= Sum < 0.06 (걷기)
            status = "걷는 중 (Walking)"
        else: # >= 0.06 (뛰기)
            status = "뛰는 중 (Running)"

    elif l_y == -1 or r_y == -1: 
        status = "측정 불가 (Ankles 안보임)"
        
    return status, debug_norm_sum


class YoloActionServer(Node):
    def __init__(self):
        super().__init__('yolo_aruco_fusion_server')
        
        # --- 1. YOLO 및 ArUco 모델 로드 ---
        self.get_logger().info("Loading YOLOv8n-Pose model...")
        self.model = YOLO('yolov8n-pose.pt') 

        self.get_logger().info("Loading ArUco detector...")
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())
        
        # ⚠️ [클라이언트 파라미터 반영] 카메라 보정 및 마커 설정
        self.MARKER_SIZE = 0.04 
        self.K = np.array([
            [610.146483, 0.0, 325.900525],
            [0.0, 609.887179, 254.935952],
            [0.0, 0.0, 1.000000]
        ], dtype=np.float32)
        self.D = np.array(
            [0.125592, -0.262183, 0.000546, -0.000613, 0.000000]
        , dtype=np.float32)


        # --- 2. 설정값 (F_STD_Sum 규칙 및 BBox Growth) ---
        self.SMOOTHING_WINDOW = 15 
        self.KEYPOINT_CONF_THRESHOLD = 0.5
        
        # 🟢 [F_STD_Sum 임계값 적용]
        self.DT_RUN_THRESHOLD = 0.06      # 걷기/뛰기 경계
        self.DT_WALK_THRESHOLD = 0.02     # 정지/걷기 경계
        
        # BBox Growth 관련 설정
        self.BBOX_AREA_GROWTH_THRESHOLD = 1.10 
        self.MIN_BBOX_AREA_FOR_CHECK = 500
        
        # --- 3. 상태 변수 초기화 ---
        self.status_history = {} 
        self.area_history = {} 
        self.FRAME_WIDTH = 640   
        self.FRAME_HEIGHT = 480  
        self.is_docking_active = False 
        
        # --- 4. ROS 2 퍼블리셔 및 Subscriber ---
        self.evasion_pub = self.create_publisher(Bool, '/trigger_evasion', 10)
        self.evasion_cooldown = False
        self.cooldown_timer = None
        
        self.docking_sub = self.create_subscription(Bool, '/docking_trigger', self.docking_callback, 10)
        self.get_logger().info("✅ /docking_trigger 구독 시작.")
        self.get_logger().info("✅ /trigger_evasion 퍼블리셔 생성됨.")
        
        # --- 5. UDP 소켓 초기화 ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 5000))
        self.BUFFER_SIZE = 65536 
        self.get_logger().info(f"UDP 수신 대기 중 (Port: 5000)...")
        self.get_logger().info(f"🔥 F_STD_Sum 규칙: Run > {self.DT_RUN_THRESHOLD}, Stop < {self.DT_WALK_THRESHOLD}, Growth > {self.BBOX_AREA_GROWTH_THRESHOLD}x")
            
    # (docking_callback 및 reset_cooldown 함수 유지)
    def docking_callback(self, msg):
        self.is_docking_active = msg.data
        if self.is_docking_active:
            self.get_logger().info("🚨 Docking Active 상태 수신.")
        else:
            self.get_logger().info("🟢 Docking Inactive 상태 수신.")
            
    def reset_cooldown(self):
        self.evasion_cooldown = False
        if self.cooldown_timer:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None

    def run_loop(self):
        """메인 실행 루프"""
        while rclpy.ok():
            try:
                # --- UDP 수신 및 디코딩 ---
                data, addr = self.sock.recvfrom(self.BUFFER_SIZE)
                np_arr = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None: continue
                frame = cv2.resize(frame, (self.FRAME_WIDTH, self.FRAME_HEIGHT))
                
                annotated_frame = frame.copy()

                # --- 1단계: ArUco 감지 및 시각화 ---
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = self.detector.detectMarkers(gray)
                aruco_display_list = []

                if ids is not None:
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE, self.K, self.D)
                    
                    for i in range(len(ids)):
                        marker_id = ids[i][0]
                        tvec = tvecs[i][0]
                        rvec = rvecs[i][0]
                        distance = np.linalg.norm(tvec)
                        
                        cv2.drawFrameAxes(annotated_frame, self.K, self.D, rvec, tvec, self.MARKER_SIZE * 0.5) 
                        aruco.drawDetectedMarkers(annotated_frame, corners)
                        
                        aruco_display_list.append(f"ArUco ID {marker_id}: {distance:.3f} m")


                # --- 2단계: YOLO-Pose 추적 및 F_STD_Sum 분석 ---
                results = self.model.track(annotated_frame, persist=True, verbose=False) 
                annotated_frame = results[0].plot() 
                
                current_ids_in_frame = set()
                yolo_status_display_list = []

                if results[0].boxes.id is not None and results[0].keypoints is not None:
                    boxes = results[0].boxes.xyxy.cpu()
                    track_ids = results[0].boxes.id.int().cpu().tolist()
                    keypoints_data = results[0].keypoints.data.cpu()

                    for box_raw, track_id, kpts_raw in zip(boxes, track_ids, keypoints_data):
                        current_ids_in_frame.add(track_id)
                        box_np = box_raw.cpu().numpy()
                        x1, y1, x2, y2 = box_np
                        current_area = (x2 - x1) * (y2 - y1)
                        
                        # BBox Growth 분석 로직
                        if track_id not in self.area_history: self.area_history[track_id] = {'last_area': current_area}
                        last_area = self.area_history[track_id].get('last_area', current_area)
                        area_growth_ratio = 1.0; is_rapidly_approaching = False 
                        if last_area > self.MIN_BBOX_AREA_FOR_CHECK and current_area > last_area:
                            area_growth_ratio = current_area / last_area
                            if area_growth_ratio >= self.BBOX_AREA_GROWTH_THRESHOLD: is_rapidly_approaching = True
                        self.area_history[track_id]['last_area'] = current_area
                        
                        # 🟢 F_STD_Sum 기반 행동 분석 (새로운 임계값 적용)
                        status, debug_norm_sum = analyze_pose_by_f_std_sum(
                            track_id, kpts_raw.data.cpu().numpy(), box_np, self.status_history, 
                            self.KEYPOINT_CONF_THRESHOLD, self.SMOOTHING_WINDOW,
                            self.DT_RUN_THRESHOLD, self.DT_WALK_THRESHOLD 
                        )
                        
                        # 📝 화면 표시 내용 구성
                        line1 = f"ID: {track_id} | 상태: {status}"
                        line2 = f"  └ F_STD_Sum: {debug_norm_sum:.4f} (R Th:{self.DT_RUN_THRESHOLD})"
                        line3 = f"  └ Approach: {area_growth_ratio:.2f}x ({is_rapidly_approaching})"
                        
                        dock_status_text = "DOCKING" if self.is_docking_active else "PATROL"
                        line4 = f"  └ System Status: {dock_status_text}"
                        
                        yolo_status_display_list.append(line1); yolo_status_display_list.append(line2); 
                        yolo_status_display_list.append(line3); yolo_status_display_list.append(line4); yolo_status_display_list.append(" ") 

                        # --- (*** ROS 2 회피 트리거 발행 ***) ---
                        is_action_detected = (status == "뛰는 중 (Running)" or status == "걷는 중 (Walking)")
                        
                        if is_action_detected and is_rapidly_approaching and not self.evasion_cooldown:
                            self.get_logger().warn(f"!!! 위협 감지: {status} + 급접근! EVASION INTERRUPT 요청 !!!")
                            self.evasion_pub.publish(Bool(data=True))
                            self.evasion_cooldown = True
                            self.cooldown_timer = self.create_timer(0.5, self.reset_cooldown) 

                # --- 7. ID 기록 삭제 ---
                lost_ids = set(self.status_history.keys()) - current_ids_in_frame
                for lost_id in lost_ids:
                    if lost_id in self.status_history: del self.status_history[lost_id]
                    if lost_id in self.area_history: del self.area_history[lost_id]

                # --- 8. 텍스트 그리기 및 출력 ---
                y_pos = 30 
                
                # ArUco 결과 출력 (왼쪽 상단)
                for aruco_text in aruco_display_list:
                    annotated_frame = draw_korean_text(annotated_frame, aruco_text, (10, y_pos), (255, 165, 0), font_size=20) 
                    y_pos += 25 
                y_pos += 10 

                # YOLO/DT 결과 출력 (이어서)
                for status_text in yolo_status_display_list:
                    text_color = (0, 0, 255)
                    if "Approach: True" in status_text: text_color = (0, 255, 255) 
                    annotated_frame = draw_korean_text(annotated_frame, status_text, (10, y_pos), text_color, font_size=20) 
                    y_pos += 25 
                
                cv2.imshow("YOLO & ArUco Fusion Server (F_STD_Sum Priority)", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                
                rclpy.spin_once(self, timeout_sec=0.001)

            except KeyboardInterrupt: break
            except Exception as e: self.get_logger().error(f"메인 루프 오류: {e}"); pass 

        self.sock.close() 
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    yolo_server_node = YoloActionServer()
    try:
        yolo_server_node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        yolo_server_node.get_logger().info("종료 및 리소스 정리...")
        yolo_server_node.destroy_node()

if __name__ == '__main__':
    main()