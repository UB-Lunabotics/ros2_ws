#!/usr/bin/env python3
"""
zed_noisy_odom.py
─────────────────
Reads ground-truth robot pose from Gazebo, adds Gaussian noise,
publishes as nav_msgs/Odometry on /zed/odom, and broadcasts
the odom → base_footprint TF.

Requires in parameter_bridge:
  /world/arena/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V
"""

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class ZedNoisyOdom(Node):
    def __init__(self):
        super().__init__('zed_noisy_odom')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('pos_noise_std',   0.01) # 0.01
        self.declare_parameter('angle_noise_std', 0.005) #0.005
        self.declare_parameter('world_frame',    'odom')
        self.declare_parameter('robot_frame',    'base_link')
        self.declare_parameter('output_topic',   '/zed/odom')
        self.declare_parameter('robot_name',     'robot')

        self.pos_std     = self.get_parameter('pos_noise_std').value
        self.ang_std     = self.get_parameter('angle_noise_std').value
        self.world_frame = self.get_parameter('world_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.robot_name  = self.get_parameter('robot_name').value
        topic            = self.get_parameter('output_topic').value

        self.pos_var = self.pos_std ** 2
        self.ang_var = self.ang_std ** 2

        # initial position stored on first message
        self.initial_pos = None

        # ── Publishers ───────────────────────────────────────────────────
        self.odom_pub       = self.create_publisher(Odometry, topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscriber (Gazebo ground truth) ─────────────────────────────
        self.create_subscription(
            TFMessage,
            '/world/arena/pose/info',
            self.pose_callback,
            10
        )

        self.get_logger().info(
            f'ZED noisy odom active → {topic}  '
            f'(pos σ={self.pos_std}m  angle σ={self.ang_std}rad)'
        )

    # ────────────────────────────────────────────────────────────────────
    def pose_callback(self, msg: TFMessage):
        # find our robot in the pose list
        robot_tf = None
        for transform in msg.transforms:
            if transform.child_frame_id == self.robot_name:
                robot_tf = transform
                break

        if robot_tf is None:
            return

        t = robot_tf.transform.translation
        q = robot_tf.transform.rotation

        # store starting position on first valid message
        if self.initial_pos is None:
            self.initial_pos = (t.x, t.y, t.z)
            self.get_logger().info(
                f'Origin set: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})'
            )
            return

        # ── Position: displacement from start + noise ────────────────────
        # odom frame is world-aligned, just offset to starting position
        dx = t.x - self.initial_pos[0] + np.random.normal(0.0, self.pos_std)
        dy = t.y - self.initial_pos[1] + np.random.normal(0.0, self.pos_std)
        dz = t.z - self.initial_pos[2] + np.random.normal(0.0, self.pos_std)

        # ── Orientation: world quaternion + noise ────────────────────────
        # odom axes are parallel to world axes so no relative rotation needed
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        r += np.random.normal(0.0, self.ang_std)
        p += np.random.normal(0.0, self.ang_std)
        y += np.random.normal(0.0, self.ang_std)
        nq = quaternion_from_euler(r, p, y)   # [x, y, z, w]

        stamp = self.get_clock().now().to_msg()

        # ── Publish Odometry ─────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp       = stamp
        odom.header.frame_id    = self.world_frame
        odom.child_frame_id     = self.robot_frame

        odom.pose.pose.position.x    = dx
        odom.pose.pose.position.y    = dy
        odom.pose.pose.position.z    = dz
        odom.pose.pose.orientation.x = nq[0]
        odom.pose.pose.orientation.y = nq[1]
        odom.pose.pose.orientation.z = nq[2]
        odom.pose.pose.orientation.w = nq[3]

        # 6x6 diagonal covariance [x, y, z, roll, pitch, yaw]
        cov = [0.0] * 36
        cov[0]  = self.pos_var
        cov[7]  = self.pos_var
        cov[14] = self.pos_var
        cov[21] = self.ang_var
        cov[28] = self.ang_var
        cov[35] = self.ang_var
        odom.pose.covariance = cov

        self.odom_pub.publish(odom)

        # ── Broadcast odom → base_footprint TF ──────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id  = self.robot_frame
        tf_msg.transform.translation.x = dx
        tf_msg.transform.translation.y = dy
        tf_msg.transform.translation.z = dz
        tf_msg.transform.rotation.x = nq[0]
        tf_msg.transform.rotation.y = nq[1]
        tf_msg.transform.rotation.z = nq[2]
        tf_msg.transform.rotation.w = nq[3]

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedNoisyOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()