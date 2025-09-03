#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SpiralDrawer(Node):
    def __init__(self):
        super().__init__('spiral_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Spiral parameters
        self.initial_radius = 0.5   # starting radius
        self.growth_rate = 0.05     # how much radius increases per step
        self.num_points = 100       # number of points in spiral
        self.speed = 2.0            # forward speed

        # Generate spiral points
        self.points = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / 10  # more cycles for spiral
            r = self.initial_radius + self.growth_rate * theta
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            self.points.append((x, y))

        self.get_logger().info(f"Generated {len(self.points)} spiral points")

        self.current_index = 0
        self.timer = self.create_timer(0.5, self.move_to_next_point)

    def move_to_next_point(self):
        if self.current_index >= len(self.points):
            self.get_logger().info("Finished drawing spiral")
            rclpy.shutdown()
            return

        target = self.points[self.current_index]
        self.get_logger().info(f"Moving to point {self.current_index}: {target}")

        # This simple version just sets constant velocities
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = 1.0  # turning rate

        self.publisher_.publish(twist)
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = SpiralDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
