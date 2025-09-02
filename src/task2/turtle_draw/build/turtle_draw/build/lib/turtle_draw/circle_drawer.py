#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')

        # publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # parameters for the circle
        self.radius = 2.0              # circle radius
        self.num_points = 36           # how many points on the circle
        self.points = []               # list to hold (x, y) pairs

        # generate list of points on a circle
        for i in range(self.num_points):
            theta = (2 * math.pi * i) / self.num_points
            x = self.radius * math.cos(theta)
            y = self.radius * math.sin(theta)
            self.points.append((x, y))

        # internal counters
        self.current_index = 0
        self.timer = self.create_timer(0.5, self.move_to_next_point)  # move every 0.5 sec

    def move_to_next_point(self):
        if self.current_index >= len(self.points):
            self.current_index = 0   # start over

        # For simplicity, just rotate slightly and move forward a bit each step.
        # (In real robotics you would compute exact velocities to reach (x, y).)
        msg = Twist()
        msg.linear.x = 1.0         # forward speed
        msg.angular.z = 0.5        # turning speed
        self.publisher.publish(msg)

        self.get_logger().info(f"Moving towards point {self.points[self.current_index]}")
        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
