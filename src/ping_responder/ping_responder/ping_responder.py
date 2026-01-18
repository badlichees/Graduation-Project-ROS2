"""
Ping Responder Node (ROS2)

该节点用于Unity-ROS2延迟测试，订阅/ping主题并立即回复/pong，实现往返延迟测量。

工作原理：
  1. 接收来自Unity的ping消息（包含时间戳和序列号）
  2. 立即将相同数据发布到/pong主题
  3. Unity端计算发送与接收的时间差得到往返延迟

主题：
  订阅: /ping (std_msgs/Float32MultiArray)
  发布: /pong (std_msgs/Float32MultiArray)

消息格式：
  data[0]: Unity时间戳（秒）
  data[1]: 序列号（整数）

注意：
  - 时间戳基准为Unity游戏时间，与ROS2系统时钟存在差异
  - 节点不修改时间戳，仅作转发，确保延迟计算准确
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PingResponder(Node):
    def __init__(self):
        super().__init__('ping_responder')
        
        # 记录节点启动时间（秒，用于相对时间计算）
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # 创建发布者和订阅者
        self.publisher = self.create_publisher(Float32MultiArray, 'pong', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ping',
            self.ping_callback,
            10
        )
        
        self.get_logger().info('Pong handler started (listening on topic "ping")')

    def ping_callback(self, msg):
        """接收ping消息并立即回复pong"""
        if len(msg.data) >= 2:
            ping_time = msg.data[0] # Unity时间戳（秒）
            sequence = int(msg.data[1])
            
            # 记录接收时间（ROS2相对时间）
            recv_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            self.get_logger().info(
                f'Received ping: seq={sequence}, ping_time={ping_time:.3f}s, '
                f'ROS2_time={recv_time:.3f}s'
            )
            
            # 立即发送pong消息，包含原始ping时间（秒）和序列号
            pong_msg = Float32MultiArray()
            pong_msg.data = [ping_time, float(sequence)]
            
            self.publisher.publish(pong_msg)
            
            # 记录发送时间并计算处理延迟
            send_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            self.get_logger().info(
                f'Sent pong: seq={sequence}, processing_delay={(send_time - recv_time)*1000:.3f}ms'
            )
        else:
            self.get_logger().warning('Received invalid ping message')


def main(args=None):
    rclpy.init(args=args)
    
    ping_responder = PingResponder()
    
    try:
        rclpy.spin(ping_responder)
    except KeyboardInterrupt:
        pass
    finally:
        ping_responder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()