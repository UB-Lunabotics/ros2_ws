import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class IRSensorPublisher(Node):
    def __init__(self):
        super().__init__('ir_sensor_publisher')
        self.publisher_ = self.create_publisher(Float32, '/ir_sensor_value', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_and_publish)  # 10Hz

    def read_and_publish(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                distance = int(line)
                msg = Float32()
                msg.data = float(distance)
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published IR distance: {distance} mm')
        except (ValueError, serial.SerialException) as e:
            self.get_logger().warn(f'Serial read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IRSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()