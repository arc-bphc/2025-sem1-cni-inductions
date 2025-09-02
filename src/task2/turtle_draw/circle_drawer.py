#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        # Create a publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Call move_in_circle every 0.1 seconds
        self.timer = self.create_timer(0.1, self.move_in_circle)

    def move_in_circle(self):
        msg = Twist()
        msg.linear.x = 1.0     # forward speed
        msg.angular.z = 0.5    # turning speed (non-zero makes a circle)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)          # keep the node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
