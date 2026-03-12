#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf') # 创建一个名为odom_to_tf的节点
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10) # 订阅 /odom 话题，队列深度为10
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) # 创建TF广播器
        self.get_logger().info('odom_to_tf节点已启动，订阅/odom')

    # 回调函数，每当 /odom 话题收到新消息时，自动调用
    def callback(self, msg):
        t = TransformStamped() # 创建空的TransformStamped消息对象
        t.header.stamp = msg.header.stamp # 时间戳同步

        # 坐标系映射：将里程计消息中的坐标系映射到TF中
        t.header.frame_id = msg.header.frame_id  # 应为 "odom"
        t.child_frame_id = msg.child_frame_id    # 应为 "base_footprint"

        # 位置提取
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation # 姿态提取（四元数表示）
        self.tf_broadcaster.sendTransform(t) # 广播TF变换，其他节点可以订阅这个变换来获取机器人在里程计坐标系中的位置和姿态

def main():
    rclpy.init()
    node = OdomToTf()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()