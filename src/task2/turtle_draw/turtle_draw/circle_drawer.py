#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        #circle params
        self.radius = 2.0
        self.num_points = 100

        #circle points
        self.points = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = 5.5 + self.radius * math.cos(theta)
            y = 5.5 + self.radius * math.sin(theta)
            self.points.append((x, y))

        self.current_index = 0
        self.pose = None
        self.timer = self.create_timer(0.01, self.move_turtle)

    def pose_callback(self, msg):
        self.pose = msg

    def move_turtle(self):
        if self.pose is None:
            return

        target_x, target_y = self.points[self.current_index]

        dx = target_x - self.pose.x
        dy = target_y - self.pose.y

        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.pose.theta

        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  #normalizing

        msg = Twist()

        if abs(angle_diff) > 0.1:
            msg.angular.z = 2.0 * angle_diff
        elif distance > 0.5:
            msg.linear.x = 2.0 * distance
        else:
            self.current_index += 1
            if self.current_index >= self.num_points:
                self.current_index = 0

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
