#!/usr/bin/env python3

"""将 /map_raw 转发为 Nav2 可用的 /map"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class MapRelay(Node):
    """用 transient_local 保留最后一张地图，避免 Nav2 后启动时拿不到地图"""

    def __init__(self):
        super().__init__('map_relay')

        sub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

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
        self.get_logger().info('map_relay 节点已启动')

    def _callback(self, msg: OccupancyGrid):
        self._pub.publish(msg)
        self.get_logger().info(
            f'地图已转发：{msg.info.width}x{msg.info.height}，分辨率={msg.info.resolution:.3f}',
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
