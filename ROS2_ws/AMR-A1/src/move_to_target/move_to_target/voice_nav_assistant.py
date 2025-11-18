import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import re
from gtts import gTTS
import os
import playsound
import tempfile

class VoiceNavAssistant(Node):
    def __init__(self):
        super().__init__('voice_nav_assistant')

        # 현재 위치 정보 구독
        self.create_subscription(String, '/current_waypoint', self.current_waypoint_callback, 10)
        
        # 도착 알림 구독
        self.create_subscription(String, '/arrival_notification', self.arrival_callback, 10)
        
        self.current_wp = "알 수 없음"

        # waypoint 명령 퍼블리셔

        self.cmd_pub = self.create_publisher(String, '/selected_waypoint', 10)

        # 음성인식 객체
        self.recognizer = sr.Recognizer()
        self.mic = None
        self.audio_available = False
        
        # 오디오 시스템 초기화

        try:
            self.mic = sr.Microphone()
            with self.mic as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.audio_available = True
            self.get_logger().info("🎙️ 음성 인식 시스템 준비 완료")
        except Exception as e:
            self.get_logger().warn(f"🔇 오디오 시스템 초기화 실패: {e}")
            self.audio_available = False

        # 주기적으로 음성 인식 시도 (오디오 사용 가능한 경우만)

        if self.audio_available:
            self.create_timer(5.0, self.listen_and_process)
            self.get_logger().info("🔊 음성 인식 활성화")
        else:
            self.get_logger().info("🔇 음성 인식 비활성화 - 텍스트 모드만 사용")

        self.get_logger().info("🎙️ 음성 보조 시스템 시작")

    def current_waypoint_callback(self, msg):
        self.current_wp = msg.data.strip()

    def arrival_callback(self, msg):
        """도착 알림 처리"""

        wp_name = msg.data.strip()
        self.speak(f"{wp_name}에 도착했습니다")
        self.get_logger().info(f"🎉 도착 알림: {wp_name}")

    def listen_and_process(self):
        """음성 인식 및 처리"""

        if not self.audio_available:
            return
            
        try:
            with self.mic as source:
                self.get_logger().info("🎧이제 말하세요...", throttle_duration_sec=3.0)
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.recognizer.listen(source, timeout=8, phrase_time_limit=5)
                text = self.recognizer.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"🗣️ 인식된 음성: {text}")
                self.process_command(text)
                
        except sr.WaitTimeoutError:
            self.get_logger().debug("⏳ 음성 입력 대기 중...", throttle_duration_sec=5.0)
        except sr.UnknownValueError:
            self.get_logger().warn("❌ 음성을 이해하지 못했습니다", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f"⚠️ 음성 인식 오류: {e}", throttle_duration_sec=5.0)

    def process_command(self, text):
        text = text.strip()

        # 1️⃣ 위치 질문

        if re.search(r"(어디|위치|지금 어디|어디에 있)", text):
            reply = f"{self.current_wp}에 있습니다."
            self.speak(reply)
            self.get_logger().info(f"💬 응답: {reply}")
            return

        # 2️⃣ 원점 이동 명령 ("원점으로 이동", "원점으로 가", "원점 가줘" 등)

        if re.search(r"원점", text):
            wp_name = "원점"
            self.get_logger().info(f"🚀 이동 명령 인식됨: {wp_name}")
            msg = String()
            msg.data = wp_name
            self.cmd_pub.publish(msg)
            self.speak(f"{wp_name}로 이동하겠습니다.")
            return

        # 3️⃣ 숫자 waypoint 명령 ("5번으로 가", "2번으로 이동", "1번까지 가줘" 등)

        match = re.search(r"(\d+)번", text)
        if match:
            wp_name = f"{match.group(1)}번"
            self.get_logger().info(f"🚀 이동 명령 인식됨: {wp_name}")
            msg = String()
            msg.data = wp_name
            self.cmd_pub.publish(msg)
            self.speak(f"{wp_name}로 이동하겠습니다.")
            return

        # 4️⃣ 인식은 됐지만 명령이 아님

        self.speak("이해하지 못했어요. 다시 말씀해 주세요.")
        self.get_logger().info("🤔 명령을 해석하지 못함.")

    def speak(self, text):
        """음성으로 말하기"""

        try:
            tts = gTTS(text=text, lang='ko')
            # 임시 파일 사용

            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as tmp_file:
                tts.save(tmp_file.name)
                playsound.playsound(tmp_file.name)
                os.unlink(tmp_file.name)
        except Exception as e:
            self.get_logger().warn(f"🔇 음성 출력 실패: {e}")
            # 음성 출력 실패 시 콘솔 출력

            print(f"📢 {text}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 음성 보조 종료")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()