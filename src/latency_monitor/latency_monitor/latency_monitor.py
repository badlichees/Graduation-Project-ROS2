"""
Latency Monitor Node (ROS2)

该节点用于监控Unity-ROS2通信的往返延迟，支持主动发送ping与被动监听pong两种模式。

工作模式：
  - 被动模式（默认）：仅订阅/pong主题，计算从Unity发送到ROS2再返回的延迟
  - 主动模式（enable_ping:=True）：定期向/ping主题发送ping，测量ROS2节点间往返延迟

参数：
  - test_interval: 主动模式下发送ping的时间间隔（秒）
  - enable_ping: 是否启用主动发送ping（默认False）

主题：
  发布: /ping (std_msgs/Float32MultiArray)
  订阅: /pong (std_msgs/Float32MultiArray)

消息格式：
  data[0]: Unity时间戳（秒）
  data[1]: 序列号（整数）

注意：
  - 在Unity-ROS2跨平台测试中，建议保持enable_ping:=False以避免干扰
  - 延迟计算使用ROS2节点时钟，与Unity时钟基准不同
"""

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import Header


class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        
        # 记录节点启动时间（秒，用于相对时间计算）
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # 创建发布者和订阅者
        self.publisher = self.create_publisher(Float32MultiArray, 'ping', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'pong',
            self.pong_callback,
            10
        )
        
        # 参数配置
        self.declare_parameter('test_interval', 1.0)  # 测试间隔（秒）
        self.declare_parameter('enable_ping', False)  # 是否启用自动发送ping
        
        self.test_interval = self.get_parameter('test_interval').get_parameter_value().double_value
        self.enable_ping = self.get_parameter('enable_ping').get_parameter_value().bool_value
        
        # 状态变量
        self.ping_time = None
        self.sequence_number = 0
        self.timer = None
        self.last_received_sequence = None  # 用于去重
        
        if self.enable_ping:
            self.timer = self.create_timer(self.test_interval, self.send_ping)
            self.get_logger().info(f'Latency tester started with interval: {self.test_interval}s')
        else:
            self.get_logger().info('Latency tester started in passive mode (no automatic ping). '
                                   'Set enable_ping:=True to activate.')

    def send_ping(self):
        """发送ping消息并记录时间戳"""
        if not self.enable_ping:
            return
        self.ping_time = self.get_clock().now().nanoseconds / 1000000.0  # 转换为毫秒
        msg = Float32MultiArray()
        msg.data = [self.ping_time, float(self.sequence_number)]
        
        self.publisher.publish(msg)
        self.get_logger().debug(f'Sent ping: seq={self.sequence_number}, time={self.ping_time:.3f}')
        self.sequence_number += 1

    def pong_callback(self, msg):
        """处理返回的pong消息并计算时钟偏移"""
        if len(msg.data) >= 2:
            received_ping_time = msg.data[0] # Unity时间戳（秒）
            sequence = int(msg.data[1])
            
            # 去重：忽略相同序列号的重复消息
            if sequence == self.last_received_sequence:
                self.get_logger().debug(f'Ignoring duplicate pong seq={sequence}')
                return
            self.last_received_sequence = sequence
            
            # 计算ROS2节点接收pong的相对时间
            recv_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            
            # 计算Unity时间戳与ROS2相对时间的偏移量
            time_offset = recv_time - received_ping_time  # 秒
            
            self.get_logger().info(
                f'Received pong: seq={sequence}, Unity_time={received_ping_time:.3f}s, '
                f'ROS2_time={recv_time:.3f}s, offset={time_offset:.3f}s'
            )
        else:
            self.get_logger().warning('Received invalid pong message')


def main(args=None):
    rclpy.init(args=args)
    
    latency_monitor = LatencyMonitor()
    
    try:
        rclpy.spin(latency_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        latency_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()