import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import socket

class IRSensorPublisher(Node):
    def __init__(self):
        super().__init__('ir_sensor_publisher')
        self.get_logger().info("I am on")
        self.publisher = self.create_publisher(Int32MultiArray, 'line_sensors', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.1.13', 3333))
        self.get_logger().info("Connected to ESP for IR sensor data")

    def read_and_publish(self):
        try:
            self.sock.settimeout(0.01)
            data = self.sock.recv(128).decode().strip()
            if data.startswith("IR:"):
                parts = data[3:].split(',')
                if len(parts) == 2:
                    left = int(parts[0])
                    right = int(parts[1])
                    msg = Int32MultiArray()
                    msg.data = [left, right]
                    self.publisher.publish(msg)
        except socket.timeout:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = IRSensorPublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.read_and_publish()
    except KeyboardInterrupt:
        pass
    node.sock.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
