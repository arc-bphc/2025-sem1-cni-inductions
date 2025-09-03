#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Parameters for circle
        self.radius = 2.0
        self.num_points = 100  # the higher, the smoother the circle
        self.speed = 1.0       # linear speed

        # Generate circle points
        self.points = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = self.radius * math.cos(theta)
            y = self.radius * math.sin(theta)
            self.points.append((x, y))

        self.get_logger().info("Circle points generated.")

        # Start drawing the circle
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.current_index = 0

    def move_turtle(self):
        msg = Twist()

        # Simple logic: move forward at constant speed while turning
        msg.linear.x = self.speed
        msg.angular.z = self.speed / self.radius

        self.publisher_.publish(msg)
        self.get_logger().info(f"Moving turtle around circle (step {self.current_index})")

        self.current_index += 1
        if self.current_index >= self.num_points:
            self.get_logger().info("Completed one circle!")
            self.current_index = 0


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
