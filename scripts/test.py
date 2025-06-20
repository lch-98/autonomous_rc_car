#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class GoalPoseListener(Node):
    def __init__(self):
        super().__init__('goal_pose_listener')
        self.get_logger().info('Goal Pose Listener node started')\
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the 2D goal pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10)

    def goal_pose_callback(self, msg):
        # Extract position and orientation from the received PoseStamped message
        x, y, _ = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        _, _, theta = self.euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        # Create a TransformStamped message
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'  # Fixed frame, change if needed
        transform_stamped.child_frame_id = 'link1'
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = 0.0  # Assuming 2D motion
        transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, \
            transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w = \
            self.quaternion_from_euler(0.0, 0.0, theta)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform_stamped)

        self.get_logger().info(f"Received goal pose: x={x}, y={y}, theta={theta}")

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return x, y, z, w


def main(args=None):
    rclpy.init(args=args)
    goal_pose_listener = GoalPoseListener()
    rclpy.spin(goal_pose_listener)
    goal_pose_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
