import rclpy
from rclpy.node import Node

from pinkylib import LED 

from pinky_interfaces.srv import SetLed, SetBrightness

class LedServiceServer(Node):
    def __init__(self):
        super().__init__('led_service_server')
        
        self.led = LED()

        self.led_service = self.create_service(SetLed, 'set_led', self.set_led_callback)
        self.brightness_service = self.create_service(SetBrightness, 'set_brightness', self.set_brightness_callback)
        self.get_logger().info('LED 제어 서비스 서버가 준비되었습니다.')

    def set_led_callback(self, request, response):
        if self.led is None:
            response.success = False
            response.message = "LED 객체가 초기화되지 않았습니다."
            return response

        command = request.command.lower()
        color = (request.r, request.g, request.b)
        
        try:
            if command == 'set_pixel':
                for pixel in request.pixels:
                    self.led.set_pixel(pixel, color)
                
                self.led.show()
                response.success = True
                response.message = f"픽셀 {request.pixels} 색상을 {color}로 변경했습니다."
            
            elif command == 'fill':
                self.led.fill(color)
                response.success = True
                response.message = f"전체 LED 색상을 {color}로 변경했습니다."

            elif command == 'clear':
                self.led.clear()
                response.success = True
                response.message = "모든 LED를 종료했습니다."
            
            else:
                response.success = False
                response.message = f"실패: 알 수 없는 명령입니다. ('set_pixel', 'fill', 'clear' 사용 가능)"

            self.get_logger().info(response.message)

        except IndexError as e:
            response.success = False
            response.message = f"실패: {str(e)}"
            self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"실패: LED 제어 중 에러 발생 - {str(e)}"
            self.get_logger().error(response.message)

        return response
   
    def set_brightness_callback(self, request, response):
        if self.led is None:
            response.success = False
            response.message = "LED 객체가 초기화되지 않았습니다."
            return response

        try:
            self.led.set_brightness(request.brightness)
            response.success = True
            response.message = f"LED 밝기를 {request.brightness}로 설정했습니다."
            self.get_logger().info(response.message)
        
        except (ValueError, Exception) as e:
            response.success = False
            response.message = f"실패: 밝기 제어 중 에러 발생 - {str(e)}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    led_service_node = LedServiceServer()
    try:
        rclpy.spin(led_service_node)
    except KeyboardInterrupt:
        pass
    finally:
        led_service_node.led.clear()
        led_service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()