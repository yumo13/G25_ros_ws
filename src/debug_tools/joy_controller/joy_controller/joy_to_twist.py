import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToTwist(Node):
    """
    ジョイスティックコマンドを Twist メッセージに変換
    """
    def __init__(self):
        super().__init__('joy_to_twist')
        
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info(f'Joy to Twist node started (max_linear: {self.max_linear_speed}, max_angular: {self.max_angular_speed})')
    
    def joy_callback(self, msg):
        twist = Twist()
        
        if len(msg.axes) > 1:
            twist.linear.y = msg.axes[1] * self.max_linear_speed
        
        if len(msg.axes) > 3:
            twist.angular.z = msg.axes[3] * self.max_angular_speed
        
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
