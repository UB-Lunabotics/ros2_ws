import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np
import tf2_ros
import math

# ─── Known tag positions in arena coordinate frame ────────────────────────────
# (x, y, z) in meters from INGRESS corner origin
TAG_POSITIONS = {
    0: (0.5,  0.0, 0.4),   # TAG 0: start of rail
    1: (1.25, 0.0, 0.4),   # TAG 1: middle of rail
    2: (2.0,  0.0, 0.4),   # TAG 2: end of rail
}

DETECTIONS_REQUIRED    = 10
POSITION_COVARIANCE    = 0.05
ORIENTATION_COVARIANCE = 0.1


class PoseInitializer(Node):
    def __init__(self):
        super().__init__('pose_initializer')
        self.get_logger().info('Pose initializer started. Searching for AprilTags...')

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.detection_accumulator = []
        self.pose_established      = False

    def tag_callback(self, msg: AprilTagDetectionArray):
        if self.pose_established:
            return
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_id = detection.id[0]

            if tag_id not in TAG_POSITIONS:
                self.get_logger().warn(f'Detected unknown tag ID {tag_id} — ignoring')
                continue

            tag_in_camera = detection.pose.pose.pose
            tx = tag_in_camera.position.x
            ty = tag_in_camera.position.y
            tz = tag_in_camera.position.z

            tag_arena_x, tag_arena_y, tag_arena_z = TAG_POSITIONS[tag_id]

            robot_arena_x = tag_arena_x - tx
            robot_arena_y = tag_arena_y - tz
            yaw = math.atan2(-tx, tz)

            self.detection_accumulator.append({
                'tag_id': tag_id,
                'x':      robot_arena_x,
                'y':      robot_arena_y,
                'yaw':    yaw,
            })

            self.get_logger().info(
                f'Tag {tag_id} detected — '
                f'estimated position: ({robot_arena_x:.2f}, {robot_arena_y:.2f}), '
                f'yaw: {math.degrees(yaw):.1f}° '
                f'[{len(self.detection_accumulator)}/{DETECTIONS_REQUIRED}]'
            )

        if len(self.detection_accumulator) >= DETECTIONS_REQUIRED:
            self.establish_pose()

    def establish_pose(self):
        xs   = [d['x']   for d in self.detection_accumulator]
        ys   = [d['y']   for d in self.detection_accumulator]
        yaws = [d['yaw'] for d in self.detection_accumulator]

        mean_x   = float(np.mean(xs))
        mean_y   = float(np.mean(ys))
        mean_yaw = float(np.mean(yaws))

        self.get_logger().info(
            f'Pose established from {len(self.detection_accumulator)} detections:\n'
            f'  Position: ({mean_x:.3f}, {mean_y:.3f})\n'
            f'  Heading:  {math.degrees(mean_yaw):.1f}°'
        )

        pose_msg                 = PoseWithCovarianceStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = mean_x
        pose_msg.pose.pose.position.y = mean_y
        pose_msg.pose.pose.position.z = 0.0

        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(mean_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(mean_yaw / 2.0)

        cov = [0.0] * 36
        cov[0]  = POSITION_COVARIANCE
        cov[7]  = POSITION_COVARIANCE
        cov[14] = 0.0
        cov[21] = 0.0
        cov[28] = 0.0
        cov[35] = ORIENTATION_COVARIANCE
        pose_msg.pose.covariance = cov

        self.pose_pub.publish(pose_msg)
        self.pose_established = True
        self.get_logger().info('Initial pose published. Pose initializer shutting down.')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseInitializer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()