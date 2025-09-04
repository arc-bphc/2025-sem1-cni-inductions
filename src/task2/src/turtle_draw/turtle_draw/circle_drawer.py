#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')

        # Publisher to turtle velocities
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to turtle pose
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.pose = None

        self.radius = 2.0
        self.center_x = 5.5
        self.center_y = 5.5
        self.num_points = 36
        self.points = self.generate_circle_points()

        self.current_goal_index = 0
        self.reached_goal = True

        self.timer = self.create_timer(0.1, self.control_loop)

    def generate_circle_points(self):
        points = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = self.center_x + self.radius * math.cos(theta)
            y = self.center_y + self.radius * math.sin(theta)
            points.append((x, y))
        return points

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            return

        goal_x, goal_y = self.points[self.current_goal_index]
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        msg = Twist()

        if distance > 0.1:
            angle_to_goal = math.atan2(dy, dx)
            angle_diff = angle_to_goal - self.pose.theta

            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            msg.linear.x = 1.5 * distance
            msg.angular.z = 6.0 * angle_diff
        else:
            self.current_goal_index = (self.current_goal_index + 1) % self.num_points

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
