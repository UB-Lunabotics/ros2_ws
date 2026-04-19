import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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
        # Wait for Nav2 to be ready
        # self.nav.waitUntilNav2Active()

        while rclpy.ok():
            # STATE 1: Go to Dig Site
            self.get_logger().info("Moving to Dig Site...")
            dig_goal = self.create_pose(2.0, 1.0) # Change to your actual coords
            self.nav.goToPose(dig_goal)
            
            while not self.nav.isTaskComplete():
                pass # Driving...

            # STATE 2: Digging
            self.get_logger().info("Starting to Dig...")
            self.bin_full = False
            while not self.bin_full:
                # Publish 'True' to start the motors
                self.dig_pub.publish(Bool(data=True))
                # Spin once to check IR sensor data
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Stop Digging
            self.dig_pub.publish(Bool(data=False))
            self.get_logger().info("Bin Full! Moving to Offload...")

            # STATE 3: Go to Offload Site
            offload_goal = self.create_pose(0.0, 0.0)
            self.nav.goToPose(offload_goal)
            
            while not self.nav.isTaskComplete():
                pass # Driving to hopper...

            # STATE 4: Offloading (Timer based)
            self.get_logger().info("Offloading...")
            # Trigger your offload mechanism here
            import time
            time.sleep(5) # Simulate offload time
            
            self.get_logger().info("Cycle Complete. Restarting...")

def main():
    rclpy.init()
    brain = LunaboticsBrain()
    brain.run_autonomy_loop()

if __name__ == '__main__':
    main()