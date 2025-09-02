import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')  # Publisher to /turtle1/cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.radius = 1.0
        self.linear_speed = 1.0
        self.angular_speed = self.linear_speed / self.radius
        self.timer = self.create_timer(0.1, self.timer_callback)  
        self.twist = Twist()
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed

    def timer_callback(self):   
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher_.publish(node.twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
