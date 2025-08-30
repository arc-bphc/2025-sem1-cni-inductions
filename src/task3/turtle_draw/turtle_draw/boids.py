#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from turtlesim.srv import Spawn


import math
import random
import numpy as np

# --- Configuration ---
NUM_TURTLES = 5
# Radius within which a turtle considers others its neighbors
RADIUS = 3.0
# Radius within which a turtle will actively try to move away from others
AVOID_RADIUS = 0.8


COHESION_WEIGHT = 0.8
ALIGNMENT_WEIGHT = 0.4
SEPARATION_WEIGHT = 1.5

MAX_SPEED = 1.5
MAX_TURN = 2.0


class boids(Node):
    def __init__(self):
        super().__init__('boids')

        self.poses = {}
        self.velocities = {}
        self.cmd_vel_publishers = []

        #Spawning Turtles
        self.spawn_turtles()

        for i in range(1, NUM_TURTLES + 1):
            turtle_name = f'turtle{i}'
            self.create_subscription(
                Pose,
                f'/{turtle_name}/pose',
                lambda msg, name=turtle_name: self.pose_callback(msg, name),
                10)
            self.cmd_vel_publishers.append(
                self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10))

        # --- Main Control Loop ---
        self.timer = self.create_timer(0.1, self.update_swarm)

    def pose_callback(self, msg, turtle_name):
        self.poses[turtle_name] = msg

        twist_msg = Twist()
        twist_msg.linear.x = msg.linear_velocity
        twist_msg.angular.z = msg.angular_velocity
        self.velocities[turtle_name] = twist_msg

    def spawn_turtles(self):

        spawner = self.create_client(Spawn, 'spawn')
        while not spawner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')

        for i in range(2, NUM_TURTLES + 1):
            req = Spawn.Request()
            req.name = f'turtle{i}'
            req.x = random.uniform(1.0, 10.0)
            req.y = random.uniform(1.0, 10.0)
            req.theta = random.uniform(0.0, 2 * math.pi)
            spawner.call_async(req)
            self.get_logger().info(f"Spawning turtle{i}")

    def update_swarm(self):
        ##############################
        if len(self.poses) < NUM_TURTLES:
            return
        ##############################

        for i in range(NUM_TURTLES):
            current_turtle_name = f'turtle{i+1}'
            current_pose = self.poses.get(current_turtle_name)
            if not current_pose:
                continue

            neighbors = self.get_neighbors(current_turtle_name)

            cohesion_vec = self.calculate_cohesion(current_pose, neighbors)
            alignment_vec = self.calculate_alignment(current_turtle_name, neighbors)
            separation_vec = self.calculate_separation(current_pose, neighbors)

            final_vec = (cohesion_vec * COHESION_WEIGHT +
                         alignment_vec * ALIGNMENT_WEIGHT +
                         separation_vec * SEPARATION_WEIGHT)

            twist_msg = Twist()
            if np.linalg.norm(final_vec) > 0:
                desired_angle = math.atan2(final_vec[1], final_vec[0])
                angle_to_rotate = desired_angle - current_pose.theta

                while angle_to_rotate > math.pi: angle_to_rotate -= 2 * math.pi
                while angle_to_rotate < -math.pi: angle_to_rotate += 2 * math.pi

                twist_msg.linear.x = min(np.linalg.norm(final_vec), MAX_SPEED)
                twist_msg.angular.z = max(min(angle_to_rotate * 2.0, MAX_TURN), -MAX_TURN)

            self.cmd_vel_publishers[i].publish(twist_msg)

    def get_neighbors(self, turtle_name):
        """Finds all turtles within the perception radius of the given turtle."""
        neighbors = []
        my_pose = self.poses[turtle_name]
        for name, pose in self.poses.items():
            if name == turtle_name:
                continue
            dist = math.sqrt((my_pose.x - pose.x)**2 + (my_pose.y - pose.y)**2)
            if dist < RADIUS:
                neighbors.append(name)
        return neighbors

    def calculate_cohesion(self, my_pose, neighbors):

        if not neighbors:
            return np.array([0.0, 0.0])

        center_of_mass = np.array([0.0, 0.0])
        for name in neighbors:
            center_of_mass += np.array([self.poses[name].x, self.poses[name].y])
        center_of_mass /= len(neighbors)
        return center_of_mass - np.array([my_pose.x, my_pose.y])

    def calculate_alignment(self, my_name, neighbors):

        if not neighbors:
            return np.array([0.0, 0.0])

        avg_velocity = np.array([0.0, 0.0])
        for name in neighbors:

            theta = self.poses[name].theta
            avg_velocity += np.array([math.cos(theta), math.sin(theta)])
        avg_velocity /= len(neighbors)
        return avg_velocity

    def calculate_separation(self, my_pose, neighbors):

        separation_vec = np.array([0.0, 0.0])
        for name in neighbors:
            neighbor_pose = self.poses[name]
            dist = math.sqrt((my_pose.x - neighbor_pose.x)**2 + (my_pose.y - neighbor_pose.y)**2)
            if dist < AVOID_RADIUS:
                repulsion = (np.array([my_pose.x, my_pose.y]) -
                             np.array([neighbor_pose.x, neighbor_pose.y]))
                if dist > 0:
                    separation_vec += repulsion / (dist * dist)
        return separation_vec

def main(args=None):
    rclpy.init(args=args)
    node = boids()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()