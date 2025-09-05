import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Circle Drawer Node has been started.')
        
        self.radius = 2.0
        self.speed = 1.0  # radians per second
        self.angular_velocity = self.speed / self.radius
        self.start_time = time.time()

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angular_velocity
        self.publisher_.publish(twist)
        self.get_logger().info('Publishing velocity command')

def main(args=None):
    rclpy.init(args=args)
    circle_drawer = CircleDrawer()
    rclpy.spin(circle_drawer)
    circle_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()