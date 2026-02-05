import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class LatencyMonitor(Node):
    """
    延迟监控节点类，发送ping消息并接收pong响应，计算时钟偏移和通信延迟
    支持自动发送或被动监听模式

    - test_interval: 自动发送ping的间隔时间（秒）
    - enable_ping: 是否启用自动发送ping
    - ping_time: 最近一次发送ping的时间戳（毫秒）
    - sequence_number: 当前序列号，用于消息匹配
    - last_received_sequence: 上次接收到的pong序列号，用于去重
    """
    def __init__(self):
        """
        初始化延迟监控节点。
        """
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
        self.declare_parameter('test_interval', 1.0)
        self.declare_parameter('enable_ping', False)
        self.test_interval = self.get_parameter('test_interval').get_parameter_value().double_value
        self.enable_ping = self.get_parameter('enable_ping').get_parameter_value().bool_value
        self.ping_time = None
        self.sequence_number = 0
        self.timer = None
        self.last_received_sequence = None
        
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
    rclpy.init(args=args) # ROS2节点中，args会被传递给节点，用于初始化ROS2上下文
    
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