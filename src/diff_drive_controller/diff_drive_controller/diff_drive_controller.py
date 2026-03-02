import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # 订阅算法输出的Twist
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 发布左右轮角速度（rad/s），供Unity接收
        self.pub_left = self.create_publisher(Float64, '/left_wheel_vel', 10)
        self.pub_right = self.create_publisher(Float64, '/right_wheel_vel', 10)
        
        # 机器人参数
        self.wheel_radius = 0.033
        self.wheel_separation = 0.16

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # 差速解算
        left_vel = (linear - angular * self.wheel_separation / 2.0) / self.wheel_radius
        right_vel = (linear + angular * self.wheel_separation / 2.0) / self.wheel_radius
        
        self.pub_left.publish(Float64(data=left_vel))
        self.pub_right.publish(Float64(data=right_vel))

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()