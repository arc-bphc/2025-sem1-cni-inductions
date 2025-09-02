import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)
        self.theta = 0.0
        self.radius = 2.0
        self.angular_speed = 0.5

    def move(self):
        msg = Twist()
        # Compute velocity components using polar-to-Cartesian conversion
        msg.linear.x = self.radius * math.cos(self.theta)
        msg.linear.y = self.radius * math.sin(self.theta)  # turtlesim ignores y, but example logic
        msg.angular.z = self.angular_speed

        self.pub.publish(msg)
        self.theta += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
