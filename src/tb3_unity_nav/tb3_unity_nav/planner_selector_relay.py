import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class PlannerSelectorRelay(Node):
    """把 Unity 的普通选择消息转成 Nav2 可保留的 planner_selector"""

    def __init__(self):
        super().__init__('planner_selector_relay')

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._pub = self.create_publisher(String, '/planner_selector', latched)
        self._sub = self.create_subscription(
            String, '/planner_selector_unity', self._cb, 10)
        self._last = ''
        self.get_logger().info('PlannerSelectorRelay ready')

    def _cb(self, msg: String):
        self._pub.publish(msg)
        if msg.data != self._last:
            self._last = msg.data
            self.get_logger().info(f'planner → {msg.data}')


def main():
    rclpy.init()
    rclpy.spin(PlannerSelectorRelay())
    rclpy.shutdown()
