#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import numpy as np
from PIL import ImageFont, ImageDraw, Image
from collections import deque
import os
import socket
import time
import threading # 🟢 스레딩 모듈 추가
from queue import Queue # 🟢 큐 모듈 추가

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
# import cv2.aruco as aruco # ArUco는 이 코드에서 사용되지 않으므로 제거

# --- 1. 설정값 (기존 유지) ---
SMOOTHING_WINDOW = 15
KEYPOINT_CONF_THRESHOLD = 0.5
DT_RUN_THRESHOLD = 0.06
DT_WALK_THRESHOLD = 0.02
BBOX_AREA_GROWTH_THRESHOLD = 1.15
MIN_BBOX_AREA_FOR_CHECK = 500
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MODEL_PATH = 'yolov8n-pose.pt' 

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
    # draw.rectangle 대신 draw.textbbox 사용
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

# --- (헬퍼 함수 3: F_STD_Sum 기반 포즈 분석) ---
def analyze_pose_by_f_std_sum(track_id, kpts, bbox, history_dict, conf_thresh, window_size, dt_run_threshold, dt_walk_threshold):
    status = "초기화 중..."
    debug_norm_sum = 0.0
    
    if track_id not in history_dict:
        history_dict[track_id] = {'left_y': deque(maxlen=window_size), 'right_y': deque(maxlen=window_size)}
        
    l_ankle = kpts[15].tolist(); r_ankle = kpts[16].tolist()
    l_y = int(l_ankle[1]) if l_ankle[2] > conf_thresh else -1
    r_y = int(r_ankle[1]) if r_ankle[2] > conf_thresh else -1
    
    # 1. 히스토리 업데이트 
    if l_y != -1: history_dict[track_id]['left_y'].append(l_y)
    else: history_dict[track_id]['left_y'].clear()
    if r_y != -1: history_dict[track_id]['right_y'].append(r_y)
    else: history_dict[track_id]['right_y'].clear()
        
    # 2. 양 발목 데이터가 모두 확보되었을 때만 분석
    if (len(history_dict[track_id]['left_y']) == window_size and 
        len(history_dict[track_id]['right_y']) == window_size):
        
        norm_factor = get_normalization_factor(bbox)
        
        # 발목 STD 계산 및 정규화
        std_left = np.std(np.array(history_dict[track_id]['left_y']))
        std_right = np.std(np.array(history_dict[track_id]['right_y']))
        
        norm_std_left = std_left / norm_factor
        norm_std_right = std_right / norm_factor
        
        # F_STD_Sum 계산
        f_std_sum = norm_std_left + norm_std_right
        debug_norm_sum = f_std_sum
        
        # 새로운 규칙 적용
        if f_std_sum < dt_walk_threshold:
            status = "정지 (Stoping)"
        elif f_std_sum < dt_run_threshold:
            status = "걷는 중 (Walking)"
        else:
            status = "뛰는 중 (Running)"

    elif l_y == -1 or r_y == -1: 
        status = "측정 불가 (Ankles 안보임)"
        
    return status, debug_norm_sum

# -----------------------------------------------------------
# 🟢 2. YOLO 추론 및 분석을 전담하는 워커 스레드
# -----------------------------------------------------------
class YoloActionWorker(threading.Thread):
    def __init__(self, input_q, output_q, server_node):
        super().__init__()
        self.input_queue = input_q
        self.output_queue = output_q
        self.node = server_node # ROS 2 로거/상태 접근용
        self.running = True
        
        # YOLO 모델은 워커 스레드에서 로드
        self.node.get_logger().info("Loading YOLOv8n-Pose model in Worker Thread...")
        self.model = YOLO(MODEL_PATH)
        
        # 상태 변수는 워커 스레드 내부에 유지
        self.status_history = {}
        self.area_history = {}
        self.is_persisted = False # YOLO 추론 persist=True 플래그
    
    def run(self):
        """워커 스레드의 메인 루프"""
        while self.running:
            try:
                # 큐에서 입력 (프레임)을 가져옵니다. (블로킹이 아니도록 timeout 설정)
                # 메인 스레드에서 UDP 프레임을 빠르게 넣어주므로, 큐가 비어있으면 0.001초 대기
                data = self.input_queue.get(timeout=0.001) 
            except Exception:
                # 큐가 비어있으면 계속 루프를 돕니다.
                continue

            frame_to_analyze = data['frame']
            
            # --- YOLO-Pose 추적 및 F_STD_Sum 분석 (고부하 작업) ---
            
            # YOLO 추론
            results = self.model.track(frame_to_analyze, persist=self.is_persisted, verbose=False)
            self.is_persisted = True # 다음 추론을 위해 persist=True 유지
            
            annotated_frame = results[0].plot() # 시각화 정보 생성
            
            # 분석 결과 초기화
            analysis_results = []
            
            if results[0].boxes.id is not None and results[0].keypoints is not None:
                boxes = results[0].boxes.xyxy.cpu()
                track_ids = results[0].boxes.id.int().cpu().tolist()
                keypoints_data = results[0].keypoints.data.cpu()

                for box_raw, track_id, kpts_raw in zip(boxes, track_ids, keypoints_data):
                    box_np = box_raw.cpu().numpy()
                    x1, y1, x2, y2 = box_np
                    current_area = (x2 - x1) * (y2 - y1)
                    
                    # BBox Growth 분석 로직
                    if track_id not in self.area_history: self.area_history[track_id] = {'last_area': current_area}
                    last_area = self.area_history[track_id].get('last_area', current_area)
                    area_growth_ratio = 1.0; is_rapidly_approaching = False 
                    if last_area > self.node.MIN_BBOX_AREA_FOR_CHECK and current_area > last_area:
                        area_growth_ratio = current_area / last_area
                        if area_growth_ratio >= self.node.BBOX_AREA_GROWTH_THRESHOLD: is_rapidly_approaching = True
                    self.area_history[track_id]['last_area'] = current_area
                    
                    # F_STD_Sum 기반 행동 분석
                    status, debug_norm_sum = analyze_pose_by_f_std_sum(
                        track_id, kpts_raw.data.cpu().numpy(), box_np, self.status_history, 
                        self.node.KEYPOINT_CONF_THRESHOLD, self.node.SMOOTHING_WINDOW,
                        self.node.DT_RUN_THRESHOLD, self.node.DT_WALK_THRESHOLD 
                    )
                    
                    # 결과 저장
                    analysis_results.append({
                        'track_id': track_id,
                        'status': status,
                        'norm_sum': debug_norm_sum,
                        'growth_ratio': area_growth_ratio,
                        'is_approaching': is_rapidly_approaching
                    })
            
            # 큐에서 ID 기록 삭제 (워커 스레드 내에서만 상태 관리)
            current_ids_in_frame = set([res['track_id'] for res in analysis_results])
            lost_ids = set(self.status_history.keys()) - current_ids_in_frame
            for lost_id in lost_ids:
                if lost_id in self.status_history: del self.status_history[lost_id]
                if lost_id in self.area_history: del self.area_history[lost_id]

            # 결과를 출력 큐에 넣습니다.
            self.output_queue.put({
                'annotated_frame': annotated_frame, 
                'analysis_results': analysis_results
            })

    def stop(self):
        self.running = False
        self.join() # 스레드가 종료될 때까지 대기

# -----------------------------------------------------------
# 🟢 3. 메인 ROS 노드 (UDP 수신 및 통신 전담)
# -----------------------------------------------------------
class YoloActionServer(Node):
    def __init__(self):
        super().__init__('yolo_evasion_fsum_server')
        
        # --- 1. 설정값 공유 ---
        self.SMOOTHING_WINDOW = 15
        self.KEYPOINT_CONF_THRESHOLD = 0.5
        self.DT_RUN_THRESHOLD = 0.06
        self.DT_WALK_THRESHOLD = 0.02
        self.BBOX_AREA_GROWTH_THRESHOLD = 1.15
        self.MIN_BBOX_AREA_FOR_CHECK = 500
        self.FRAME_WIDTH = 640   
        self.FRAME_HEIGHT = 480  
        
        # --- 2. 스레드 및 큐 설정 ---
        self.input_queue = Queue(maxsize=1) # 입력 큐 크기 1로 설정 (가장 최신 프레임만 유지)
        self.output_queue = Queue(maxsize=1) # 출력 큐 크기 1로 설정 (가장 최신 결과만 유지)
        
        # 워커 스레드 시작
        self.worker = YoloActionWorker(self.input_queue, self.output_queue, self)
        self.worker.start()
        self.get_logger().info("✅ YOLO Action Worker 스레드 시작됨.")
        
        # --- 3. 상태 변수 및 ROS 통신 (메인 스레드 유지) ---
        self.is_docking_active = False
        self.evasion_cooldown = False
        self.cooldown_timer = None
        
        self.evasion_pub = self.create_publisher(Bool, '/trigger_evasion', 10)
        self.docking_sub = self.create_subscription(Bool, '/docking_trigger', self.docking_callback, 10)
        
        # --- 4. UDP 소켓 초기화 ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 5000))
        self.BUFFER_SIZE = 65536
        self.sock.settimeout(0.1) # 0.1초 타임아웃
        self.get_logger().info(f"UDP 수신 대기 중 (Port: 5000)...")
            
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
        """메인 실행 루프 (UDP 수신 및 ROS 통신 전담)"""
        while rclpy.ok():
            try:
                # --- 1. UDP 수신 (메인 스레드) ---
                data, addr = self.sock.recvfrom(self.BUFFER_SIZE)
                np_arr = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None: continue
                frame = cv2.resize(frame, (self.FRAME_WIDTH, self.FRAME_HEIGHT))
                
                # --- 2. 워커 스레드에 프레임 전달 ---
                # 큐가 가득 차 있다면 (워커가 아직 이전 프레임 처리를 끝내지 못했다면), 이전 프레임은 버리고 최신 프레임으로 대체
                if self.input_queue.full():
                    try: self.input_queue.get_nowait()
                    except Exception: pass
                self.input_queue.put({'frame': frame.copy()}) 
                
                annotated_frame = frame.copy()
                yolo_status_display_list = []
                is_evasion_triggered = False

                # --- 3. 워커 스레드의 최신 결과 가져오기 ---
                try:
                    latest_result = self.output_queue.get_nowait()
                    
                    annotated_frame = latest_result['annotated_frame']
                    analysis_results = latest_result['analysis_results']
                    
                    # --- 4. ROS 2 회피 트리거 발행 (분석 결과 기반) ---
                    for res in analysis_results:
                        status = res['status']
                        debug_norm_sum = res['norm_sum']
                        area_growth_ratio = res['growth_ratio']
                        is_rapidly_approaching = res['is_approaching']
                        track_id = res['track_id']
                        
                        # 📝 화면 표시 내용 구성
                        dock_status_text = "DOCKING" if self.is_docking_active else "PATROL"
                        yolo_status_display_list.append(f"ID: {track_id} | 상태: {status}")
                        yolo_status_display_list.append(f"  └ F_STD_Sum: {debug_norm_sum:.4f} (R Th:{self.DT_RUN_THRESHOLD})")
                        yolo_status_display_list.append(f"  └ Approach: {area_growth_ratio:.2f}x ({is_rapidly_approaching})")
                        yolo_status_display_list.append(f"  └ System Status: {dock_status_text}")
                        yolo_status_display_list.append(" ") 
                        
                        is_action_detected = (status == "뛰는 중 (Running)" or status == "걷는 중 (Walking)")
                        
                        if is_action_detected and is_rapidly_approaching and not self.evasion_cooldown:
                            self.get_logger().warn(f"!!! 위협 감지: {status} + 급접근! EVASION INTERRUPT 요청 !!!")
                            self.evasion_pub.publish(Bool(data=True))
                            self.evasion_cooldown = True
                            self.cooldown_timer = self.create_timer(0.5, self.reset_cooldown)
                            is_evasion_triggered = True

                except Exception:
                    # 큐가 비어있는 경우 (워커가 아직 결과를 못 만든 경우)
                    pass
                
                # --- 5. 텍스트 그리기 및 출력 (메인 스레드) ---
                y_pos = 30
                # F_STD_Sum 분석 결과 출력
                for status_text in yolo_status_display_list:
                    text_color = (0, 0, 255)
                    if "Approach: True" in status_text: text_color = (0, 255, 255)
                    annotated_frame = draw_korean_text(annotated_frame, status_text, (10, y_pos), text_color, font_size=20)
                    y_pos += 25
                
                # 프레임이 밀려서 최신 분석 결과가 없는 경우 현재 상태만 출력
                if not yolo_status_display_list:
                    annotated_frame = draw_korean_text(annotated_frame, f"Worker 상태: 대기 중", (10, 30), (0, 255, 255), font_size=20)

                cv2.imshow("YOLO Evasion Server (Multi-threaded)", annotated_frame)
                
                # --- 6. ROS 및 OpenCV 이벤트 처리 (메인 스레드) ---
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                rclpy.spin_once(self, timeout_sec=0.001)

            except socket.timeout:
                # UDP 타임아웃 발생 시, ROS 이벤트만 처리하고 다음 수신 대기
                rclpy.spin_once(self, timeout_sec=0.001)
                pass 
            except KeyboardInterrupt: break
            except Exception as e: 
                self.get_logger().error(f"메인 루프 오류: {e}"); time.sleep(0.1); pass 

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
        yolo_server_node.get_logger().info("워커 스레드 종료 요청...")
        yolo_server_node.worker.stop() # 🟢 워커 스레드 종료
        yolo_server_node.get_logger().info("종료 및 리소스 정리 완료.")
        yolo_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()