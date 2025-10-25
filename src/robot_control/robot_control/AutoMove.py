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
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.2
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        self.connected = False
    def compute_pid(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        self.integral += error * dt
        self.integral = max(min(self.integral, 10.0), -10.0)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative
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
        error = right - left
        correction = self.compute_pid(error)
        twist = Twist()

        if left == 0 and right == 0:
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.3
            twist.angular.z = correction

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
