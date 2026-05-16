import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class IRSensorPublisher(Node):
    def __init__(self):
        super().__init__('ir_sensor_publisher')
        self.publisher_ = self.create_publisher(Float32, '/ir_sensor_value', 10)
        self.serial_port = None
        self.timer = self.create_timer(0.1, self.read_and_publish)  # 10Hz
        self.connect_serial()

    def connect_serial(self):
        """Try to connect to serial port, retry until successful."""
        while rclpy.ok():
            try:
                self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
                self.get_logger().info('IR sensor connected on /dev/ttyUSB0')
                return
            except serial.SerialException:
                self.get_logger().warn(
                    'IR sensor not found on /dev/ttyUSB0, retrying in 3s...'
                )
                time.sleep(3)

    def read_and_publish(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.connect_serial()
            return
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
            self.serial_port = None  # will trigger reconnect on next tick

def main(args=None):
    rclpy.init(args=args)
    node = IRSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()