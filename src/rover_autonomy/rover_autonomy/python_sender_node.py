import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
import serial
import struct

DRUM_FULL_THRESHOLD_MM = 100  # mm

class JetsonSerialSender(Node):
    def __init__(self):
        super().__init__('jetson_serial_sender')

        # Serial port to ESP32
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Latest values
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.drum_mode = False

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.ir_sub = self.create_subscription(
            UInt32, '/ir_distance', self.ir_callback, 10)

        # Send at 10Hz
        self.timer = self.create_timer(0.1, self.send_packet)

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def ir_callback(self, msg: UInt32):
        # false = dig, true = dump
        self.drum_mode = msg.data < DRUM_FULL_THRESHOLD_MM

    def send_packet(self):
        # Pack into binary: start byte, 2 floats, 1 bool, end byte
        packet = struct.pack(
            '>BffBB',        # big-endian: byte, float, float, byte, byte
            0xFF,            # start byte
            self.linear_x,   # 4 bytes
            self.angular_z,  # 4 bytes
            self.drum_mode,  # 1 byte (0 or 1)
            0xFE             # end byte
        )
        self.serial_port.write(packet)
        self.get_logger().info(
            f'Sent → linear_x: {self.linear_x}, angular_z: {self.angular_z}, drum_full: {self.drum_mode}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = JetsonSerialSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()