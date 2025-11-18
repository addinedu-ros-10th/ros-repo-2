import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pinkylib import Battery 

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publihser')
        
        self.battery = Battery()

        self.percentage_publisher = self.create_publisher(
            Float32,
            'battery/present',
            10
        )
        
        self.voltage_publisher = self.create_publisher(
            Float32,
            'battery/voltage',
            10
        )

        self.timer_period = 5.0
        self.percentage_timer = self.create_timer(self.timer_period, self.percentage_callback)
        self.voltage_timer = self.create_timer(self.timer_period, self.voltage_callback)

    def percentage_callback(self):
        pct_msg = Float32()
        pct_msg.data = float(self.battery.battery_percentage())
        self.percentage_publisher.publish(pct_msg)

    def voltage_callback(self):
        volt_msg = Float32()
        volt_msg.data = float(self.battery.get_voltage()) 
        self.voltage_publisher.publish(volt_msg)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = BatteryPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
