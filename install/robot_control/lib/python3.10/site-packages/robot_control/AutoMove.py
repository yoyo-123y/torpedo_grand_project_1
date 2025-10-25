import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import time
class AutoMove(Node):
    def __init__(self):
        super().__init__('automove')
        self.subscription = self.create_subscription(Int32MultiArray, 'line_sensors', self.follow_line, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wait_for_topic('line_sensors', timeout=10.0)

        self.connected = False
    def wait_for_topic(self, topic_name, timeout=10.0):
        start_time = time.time()

        while rclpy.ok():
            topics = [t[0] for t in self.get_topic_names_and_types()]
            if topic_name in topics:
                return True
            if (time.time() - start_time > timeout):
                return False
            time.sleep(0.5)

    def follow_line(self,msg):
        left, right = msg.data
        twist = Twist()

        if left == 0 and right == 0:
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        elif left == 0 and right == 1:
            twist.linear.x = 0.2
            twist.angular.z = 0.5
        elif left == 1 and right == 0:
            twist.linear.x = 0.2
            twist.angular.z = -0.5
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)
        self.get_logger().info(f"Line state L={left} R={right}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
