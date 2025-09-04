import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.draw_circle)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def draw_circle(self):
        msg = Twist()
        msg.linear.x = 1.0   # move forward
        msg.angular.z = 1.0  # turn while moving
        self.publisher.publish(msg)

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.start_time > 6.5:  # stop after ~1 circle
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)

