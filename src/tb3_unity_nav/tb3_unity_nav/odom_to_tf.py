#!/usr/bin/env python3

"""将 Unity 发布的 /odom 补成 Nav2 需要的 TF"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToTf(Node):
    """复用里程计中的 frame，避免额外维护一份坐标关系"""

    def __init__(self):
        super().__init__('odom_to_tf')
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info('odom_to_tf 节点已启动')

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
