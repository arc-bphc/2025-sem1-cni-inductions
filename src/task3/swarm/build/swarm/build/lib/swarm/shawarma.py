#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Parameters you can tune
PERCEPTION_RADIUS = 3.0
W_SEPARATION = 2.0
W_ALIGNMENT = 1.0
W_COHESION = 0.5
FORWARD_SPEED = 1.5

class BoidController(Node):
    def __init__(self, turtle_names):
        super().__init__('boid_controller')
        self.turtle_names = turtle_names
        self.poses = {name: None for name in turtle_names}
        self.fug_publishers = {}
        for name in turtle_names:
            self.create_subscription(
                Pose,
                f'/{name}/pose',
                lambda msg, name=name: self.pose_callback(msg, name),
                10
            )
            self.fug_publishers[name] = self.create_publisher(
                Twist,
                f'/{name}/cmd_vel',
                10
            )
        self.timer = self.create_timer(0.1, self.update)

    def pose_callback(self, msg, name):
        self.poses[name] = msg

    def update(self):
        for name in self.turtle_names:
            pose = self.poses[name]
            if pose is None:
                continue

            neighbors = [
                (n, self.poses[n])
                for n in self.turtle_names
                if n != name and self.poses[n] is not None
                and self.distance(pose, self.poses[n]) < PERCEPTION_RADIUS
            ]

            if not neighbors:
                continue

            # Compute separation, alignment, cohesion vectors
            sep_x, sep_y = 0.0, 0.0
            avg_heading = 0.0
            avg_x, avg_y = 0.0, 0.0

            for _, n_pose in neighbors:
                dx = pose.x - n_pose.x
                dy = pose.y - n_pose.y
                dist = math.sqrt(dx**2 + dy**2)
                if dist > 0:
                    sep_x += dx / dist
                    sep_y += dy / dist
                avg_heading += n_pose.theta
                avg_x += n_pose.x
                avg_y += n_pose.y

            count = len(neighbors)
            avg_heading /= count
            avg_x /= count
            avg_y /= count

            # Combine weighted steering
            steer_x = (W_SEPARATION * sep_x +
                       W_COHESION * (avg_x - pose.x))
            steer_y = (W_SEPARATION * sep_y +
                       W_COHESION * (avg_y - pose.y))

            # Alignment (adjust heading toward average heading)
            desired_heading = math.atan2(steer_y, steer_x)
            heading_error = desired_heading - pose.theta
            # normalize heading
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            twist = Twist()
            twist.linear.x = FORWARD_SPEED
            twist.angular.z = W_ALIGNMENT * heading_error
            self.publishers[name].publish(twist)

    @staticmethod
    def distance(p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def main(args=None):
    rclpy.init(args=args)
    # Replace with the turtles you spawned in Task 2 i totally typed this are you reading this coolcool
    turtle_names = ['turtle1', 'turtle2', 'turtle3']
    node = BoidController(turtle_names)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
