#!/usr/bin/env python3
"""
map_relay.py
将 Unity 通过 ros_tcp_endpoint 发布的 /map_raw（volatile QoS）
转发为 Nav2 所需的 /map（transient_local QoS）。

QoS 说明：
  ros_tcp_endpoint 发布：reliable + volatile
  Nav2 (AMCL / costmap static_layer) 订阅：reliable + transient_local
  两者 durability 不匹配会导致消息被丢弃，此节点作为桥接层。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')

        # 订阅 /map_raw：volatile，与 ros_tcp_endpoint 匹配
        sub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 发布 /map：transient_local，与 Nav2 的 AMCL / costmap 匹配
        pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(OccupancyGrid, '/map', pub_qos)
        self._sub = self.create_subscription(
            OccupancyGrid, '/map_raw', self._callback, sub_qos
        )
        self.get_logger().info('map_relay 已启动：/map_raw (volatile) → /map (transient_local)')

    def _callback(self, msg: OccupancyGrid):
        self._pub.publish(msg)
        self.get_logger().info(
            f'地图已转发：{msg.info.width}×{msg.info.height}，分辨率={msg.info.resolution:.3f}m',
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
