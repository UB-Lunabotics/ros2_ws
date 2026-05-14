import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32, Bool
import time
import threading

class MockRover(Node):
    def __init__(self):
        super().__init__('mock_rover')
        self.get_logger().info("Mock Rover Online. Waiting for commands...")

        # 1. Fake Nav2 Action Server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_nav_callback
        )

        # 2. Fake IR Sensor Publisher
        self.ir_pub = self.create_publisher(Float32, '/ir_sensor_value', 10)
        self.digging_active = False

        # 3. Subscriber to listen to the Brain's dig commands
        self.create_subscription(Bool, '/cmd_dig', self.dig_cmd_callback, 10)

        # Start a background thread to publish sensor data
        self.sensor_thread = threading.Thread(target=self.publish_fake_sensor_data)
        self.sensor_thread.start()

    def dig_cmd_callback(self, msg):
        self.digging_active = msg.data
        if self.digging_active:
            self.get_logger().info("[HARDWARE MOCK] Excavation motors TURNED ON.")
        else:
            self.get_logger().info("[HARDWARE MOCK] Excavation motors TURNED OFF.")

    def execute_nav_callback(self, goal_handle):
        target_x = goal_handle.request.pose.pose.position.x
        target_y = goal_handle.request.pose.pose.position.y
        
        self.get_logger().info(f"[NAV2 MOCK] Driving to X: {target_x}, Y: {target_y}...")
        
        # Simulate the time it takes to drive
        time.sleep(4.0) 
        
        self.get_logger().info("[NAV2 MOCK] Arrived at destination.")
        goal_handle.succeed()
        
        result = NavigateToPose.Result()
        return result

    def publish_fake_sensor_data(self):
        # Starts at 0. When digging starts, it slowly fills up.
        current_fill = 0.0
        while rclpy.ok():
            if self.digging_active:
                current_fill += 2.5 # Simulate dirt filling the bin
                self.get_logger().info(f"[SENSOR MOCK] Bin level: {current_fill}")
            else:
                if current_fill > 10.0:
                    current_fill = 0.0 # Reset bin after offloading

            msg = Float32()
            msg.data = current_fill
            self.ir_pub.publish(msg)
            time.sleep(1.0) # Publish 1Hz

def main(args=None):
    rclpy.init(args=args)
    mock_rover = MockRover()
    rclpy.spin(mock_rover)

if __name__ == '__main__':
    main()