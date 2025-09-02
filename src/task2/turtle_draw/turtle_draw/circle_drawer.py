import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_circle)
        self.theta_dot = 0.5  # angular speed
        self.radius = 2.0
        self.theta = 0.0

    def move_circle(self):
        msg = Twist()
        msg.linear.x = self.radius * self.theta_dot  # v = r * theta_dot
        msg.angular.z = self.theta_dot               # omega = theta_dot
        self.publisher_.publish(msg)
        self.theta += self.theta_dot * 0.1  # update theta for next step

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
