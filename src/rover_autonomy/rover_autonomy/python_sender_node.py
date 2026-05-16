import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
import serial
import struct
import time

DRUM_FULL_THRESHOLD_MM = 100  # mm

class JetsonSerialSender(Node):
    def __init__(self):
        super().__init__('jetson_serial_sender')

        self.serial_port = None

        # Latest values
        self.linear_x  = 0.0
        self.angular_z = 0.0
        self.drum_mode = False

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.ir_sub = self.create_subscription(
            UInt32, '/ir_distance', self.ir_callback, 10)

        # Send at 10Hz
        self.timer = self.create_timer(0.1, self.send_packet)

        # Try to connect
        self.connect_serial()

    def connect_serial(self):
        """Try to connect to ESP32 serial port, retry until successful."""
        while rclpy.ok():
            try:
                self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
                self.get_logger().info('ESP32 connected on /dev/ttyUSB0')
                return
            except serial.SerialException:
                self.get_logger().warn(
                    'ESP32 not found on /dev/ttyUSB0, retrying in 3s...'
                )
                time.sleep(3)

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x  = msg.linear.x
        self.angular_z = msg.angular.z

    def ir_callback(self, msg: UInt32):
        self.drum_mode = msg.data < DRUM_FULL_THRESHOLD_MM

    def send_packet(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.connect_serial()
            return

        try:
            packet = struct.pack(
                '>BffBB',
                0xFF,
                self.linear_x,
                self.angular_z,
                self.drum_mode,
                0xFE
            )
            self.serial_port.write(packet)
            self.get_logger().info(
                f'Sent → linear_x: {self.linear_x:.2f}, '
                f'angular_z: {self.angular_z:.2f}, '
                f'drum_full: {self.drum_mode}'
            )
        except serial.SerialException as e:
            self.get_logg