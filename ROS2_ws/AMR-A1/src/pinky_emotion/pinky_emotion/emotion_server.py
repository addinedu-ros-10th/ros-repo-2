import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from PIL import Image, ImageSequence
import os
import threading

from pinky_interfaces.srv import Emotion
from .pinky_lcd import LCD

class PinkyEmotion(Node):
    def __init__(self):
        super().__init__('pinky_emotion')

        self.emotion_path = os.path.join(get_package_share_directory('pinky_emotion'), 'emotion')
        self.emotion_service = self.create_service(Emotion, 'set_emotion', self.set_emotion_callback)
        self.lcd = LCD()
        
        self.gif_frames = []
        self.current_frame_index = 0
        self.gif_lock = threading.Lock() 
        
        self.emotion_cache = {}  
        self._preload_gifs()

        self.animation_timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Pinky's emotion server is ready!! All GIFs pre-loaded.")

        with self.gif_lock:
            self.gif_frames = self.emotion_cache.get("basic", [])

    def _preload_gifs(self):
        self.get_logger().info("Pre-loading all emotion GIFs into memory...")
        try:
            gif_files = [f for f in os.listdir(self.emotion_path) if f.endswith('.gif')]
            for gif_file in gif_files:
                emotion_name = os.path.splitext(gif_file)[0]
                file_path = os.path.join(self.emotion_path, gif_file)
                
                img = Image.open(file_path)
                frames = []
                for i, frame in enumerate(ImageSequence.Iterator(img)):
                    if i % 2 == 0: 
                        frames.append(frame.copy().convert("RGB"))
                
                self.emotion_cache[emotion_name] = frames
                self.get_logger().info(f"  - Cached '{emotion_name}' ({len(frames)} frames)")
        except Exception as e:
            self.get_logger().error(f"Failed during GIF pre-loading: {e}")

    def set_emotion_callback(self, request, response):
        emo = request.emotion
        self.get_logger().info(f"Request to set emotion to '{emo}'")

        if emo in self.emotion_cache:
            with self.gif_lock:
                self.gif_frames = self.emotion_cache[emo]
                self.current_frame_index = 0
            response.response = f"Emotion set to {emo}"
        else:
            response.response = "Wrong command or emotion not cached"
            self.get_logger().warn(f"Emotion '{emo}' not found in cache.")

        return response

    def timer_callback(self):
        with self.gif_lock:
            if not self.gif_frames:
                return

            frame_to_show = self.gif_frames[self.current_frame_index]
            self.lcd.img_show(frame_to_show)

            self.current_frame_index = (self.current_frame_index + 1) % len(self.gif_frames)


def main(args=None):
    rclpy.init(args=args)
    pinky_emotion_node = PinkyEmotion()
     
    try:
        rclpy.spin(pinky_emotion_node)
    except KeyboardInterrupt:
        pinky_emotion_node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        pinky_emotion_node.lcd.clear()
        pinky_emotion_node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()