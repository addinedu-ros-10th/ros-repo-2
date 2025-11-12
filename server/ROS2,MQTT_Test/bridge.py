import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import paho.mqtt.client as mqtt
import struct

class ROSMQTTBridge(Node):
    def __init__(self):
        super().__init__('ros_mqtt_bridge')
        self.create_subscription(Point, '/robot1/pos', self.robot1_callback, 10)
        self.create_subscription(Point, '/robot2/pos', self.robot2_callback, 10)
        self.create_subscription(Point, '/robot3/pos', self.robot3_callback, 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("192.168.0.184", 1883, 60)
        self.mqtt_client.subscribe("ai/object")
        self.mqtt_client.loop_start()

        print("[Bridge 시작] ROS2와 MQTT 데이터 통합 중...")

    def robot1_callback(self, msg):
        print(f"[ROS2 수신] 로봇1 위치: ({msg.x:.2f}, {msg.y:.2f})")

    def robot2_callback(self, msg):
        print(f"[ROS2 수신] 로봇2 위치: ({msg.x:.2f}, {msg.y:.2f})")

    def robot3_callback(self, msg):
        print(f"[ROS2 수신] 로봇3 위치: ({msg.x:.2f}, {msg.y:.2f})")

    def on_mqtt_message(self, client, userdata, msg):
        data = msg.payload
        id = data[0]
        sensor = data[1:3].decode(errors='ignore')
        led_status = struct.unpack('fff', data[3:15])
        print(f"[TCP 수신] ID={id}, 센서={sensor}, LED 상태값=({led_status})")

def main():
    rclpy.init()
    node = ROSMQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
