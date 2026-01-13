import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')
        
        # Subscriber for joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publisher for cmd_vel topic
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Joy to Twist node started')
    
    def joy_callback(self, msg):
        # Create Twist message
        twist = Twist()
        
        # Axis 2 (index 1) -> linear.y
        if len(msg.axes) > 1:
            twist.linear.y = msg.axes[1]
        
        # Axis 4 (index 3) -> angular.z
        if len(msg.axes) > 3:
            twist.angular.z = msg.axes[3]
        
        # Publish the Twist message
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
