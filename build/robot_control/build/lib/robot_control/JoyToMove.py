import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("✅ JoyToTwist node started — move the left stick to drive the turtle!")

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[1] * 1.0  # forward/backward
        twist.angular.z = msg.axes[0] * 1.0  # left/right rotation
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()