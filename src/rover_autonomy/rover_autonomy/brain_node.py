import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class LunaboticsBrain(Node):
    def __init__(self):
        super().__init__('lunabotics_brain')
        
        # 1. Setup Navigation
        self.nav = BasicNavigator()
        
        # 2. Setup IR Sensor Subscriber
        self.bin_full = False
        self.create_subscription(Float32, '/ir_sensor_value', self.ir_callback, 10)
        
        # 3. Setup Digging Command Publisher
        self.dig_pub = self.create_publisher(Bool, '/cmd_dig', 10)

    def ir_callback(self, msg):
        # If reading is above threshold (10), mark as full
        if msg.data >= 10.0:
            self.bin_full = True

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def run_autonomy_loop(self):
        # STATE 1: Dig immediately in starting zone
        self.get_logger().info("Digging in starting zone...")
        self.bin_full = False
        while not self.bin_full:
            self.dig_pub.publish(Bool(data=True))
            rclpy.spin_once(self, timeout_sec=0.1)
        self.dig_pub.publish(Bool(data=False))

        while rclpy.ok():
            # STATE 2: Navigate to berm and dump
            self.get_logger().info("Navigating to berm...")
            self.nav.goToPose(self.create_pose(6.80, 3.57))
            while not self.nav.isTaskComplete():
                pass
            self.get_logger().info("Dumping...")
            time.sleep(5)

            # STATE 3: Navigate to excavation center and dig
            self.get_logger().info("Navigating to dig site...")
            self.nav.goToPose(self.create_pose(2.0, 2.285))
            while not self.nav.isTaskComplete():
                pass
            self.get_logger().info("Digging...")
            self.bin_full = False
            while not self.bin_full:
                self.dig_pub.publish(Bool(data=True))
                rclpy.spin_once(self, timeout_sec=0.1)
            self.dig_pub.publish(Bool(data=False))

            self.get_logger().info("Cycle complete. Repeating...")

def main():
    rclpy.init()
    brain = LunaboticsBrain()
    brain.run_autonomy_loop()

if __name__ == '__main__':
    main()