#!/usr/bin/env python3
"""Publish a TF transform from a PoseStamped topic."""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformBroadcaster


class PoseStampedTfBridgeNode(Node):
    def __init__(self):
        super().__init__('pose_stamped_tf_bridge')
        self.declare_parameter('input_pose', '/glim_ros/pose_corrected')
        self.declare_parameter('child_frame_id', 'glim_imu')
        self.declare_parameter('parent_frame_id_override', '')

        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.parent_frame_id_override = self.get_parameter(
            'parent_frame_id_override').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            PoseStamped,
            self.get_parameter('input_pose').value,
            self.on_pose,
            20)
        self.get_logger().info(
            f'pose_stamped_tf_bridge started: '
            f"{self.get_parameter('input_pose').value} -> {self.child_frame_id}")

    def on_pose(self, msg):
        tf = TransformStamped()
        tf.header = msg.header
        if self.parent_frame_id_override:
            tf.header.frame_id = self.parent_frame_id_override
        tf.child_frame_id = self.child_frame_id
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation
        self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = PoseStampedTfBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
