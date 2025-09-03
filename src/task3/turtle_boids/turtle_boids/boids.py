#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import random
from functools import partial


class BoidsSwarm(Node):
    def __init__(self, num_boids: int = 6):
        super().__init__('boids_swarm')

        self.num_boids = max(5, int(num_boids))  # require at least 5
        self.names = []         # actual names returned by spawn service
        self.poses = {}         # map name -> latest Pose (turtlesim.msg.Pose)
        self.pubs = {}          # map name -> cmd_vel publisher

        # Spawn service client
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # Spawn turtles at random positions (avoid walls)
        for i in range(self.num_boids):
            req = Spawn.Request()
            req.x = random.uniform(2.0, 9.0)
            req.y = random.uniform(2.0, 9.0)
            req.theta = random.uniform(-math.pi, math.pi)
            # request a friendly name, but use the returned name to be safe
            req.name = f'boid{i+1}'
            future = self.spawn_client.call_async(req)
            # wait for spawn to finish
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            try:
                resp = future.result()
                name = resp.name
            except Exception as e:
                self.get_logger().error(f'Error spawning boid {i+1}: {e}')
                name = req.name
            self.get_logger().info(f'Spawned {name} at ({req.x:.2f}, {req.y:.2f})')
            self.names.append(name)

        # Create publishers & subscribers for each boid
        for name in self.names:
            cmd_topic = f'/{name}/cmd_vel'
            pose_topic = f'/{name}/pose'
            self.pubs[name] = self.create_publisher(Twist, cmd_topic, 10)
            # subscriber with partial to store pose in dict
            self.create_subscription(Pose, pose_topic, partial(self._pose_cb, name), 10)
            self.poses[name] = None

        # Boids parameters (tweak these to taste)
        self.max_linear = 2.0
        self.max_angular = 3.0
        self.neighbor_dist = 2.0
        self.sep_weight = 1.5
        self.align_weight = 1.0
        self.coh_weight = 1.0
        self.boundary_weight = 4.0

        # turtlesim area center & margins for boundary avoidance
        self.center_x = 5.5
        self.center_y = 5.5
        self.margin = 1.0

        # control loop timer
        self.timer_period = 0.1  # seconds (10 Hz)
        self.create_timer(self.timer_period, self._control_loop)

        self.get_logger().info(f'BoidsSwarm started with {len(self.names)} boids.')

    def _pose_cb(self, name: str, msg: Pose):
        """Store latest pose for turtle `name`."""
        self.poses[name] = msg

    def _neighbors(self, name: str):
        """Return list of (other_name, Pose, dist) within neighbor_dist."""
        p = self.poses.get(name)
        if p is None:
            return []
        neighbors = []
        for other in self.names:
            if other == name:
                continue
            op = self.poses.get(other)
            if op is None:
                continue
            dx = op.x - p.x
            dy = op.y - p.y
            dist = math.hypot(dx, dy)
            if dist <= self.neighbor_dist:
                neighbors.append((other, op, dist))
        return neighbors

    @staticmethod
    def _limit(val, low, high):
        return max(low, min(high, val))

    def _control_loop(self):
        """Main control loop: compute boids steering and publish Twist for each turtle."""
        for name in self.names:
            pose = self.poses.get(name)
            if pose is None:
                # no pose yet; skip this boid
                continue

            xi, yi, thetai = pose.x, pose.y, pose.theta

            neighbors = self._neighbors(name)

            # initialize steering components
            sep_x = sep_y = 0.0
            align_x = align_y = 0.0
            coh_x = coh_y = 0.0

            if neighbors:
                # Cohesion: move toward average neighbor position
                avg_x = sum(p.x for (_, p, _) in neighbors) / len(neighbors)
                avg_y = sum(p.y for (_, p, _) in neighbors) / len(neighbors)
                coh_x = (avg_x - xi)
                coh_y = (avg_y - yi)

                # Alignment: average heading vector of neighbors
                avg_vx = sum(math.cos(p.theta) for (_, p, _) in neighbors) / len(neighbors)
                avg_vy = sum(math.sin(p.theta) for (_, p, _) in neighbors) / len(neighbors)
                align_x = avg_vx
                align_y = avg_vy

                # Separation: away from close neighbors (stronger when very close)
                for (_, p, dist) in neighbors:
                    if dist > 1e-6:
                        sep_x += (xi - p.x) / (dist * dist)
                        sep_y += (yi - p.y) / (dist * dist)

            # Boundary avoidance: steer away from walls / toward center when near edges
            bx = by = 0.0
            xmin, xmax, ymin, ymax = 0.5, 10.5, 0.5, 10.5
            if xi < xmin + self.margin:
                bx += (xmin + self.margin - xi)
            elif xi > xmax - self.margin:
                bx -= (xi - (xmax - self.margin))
            if yi < ymin + self.margin:
                by += (ymin + self.margin - yi)
            elif yi > ymax - self.margin:
                by -= (yi - (ymax - self.margin))

            # Combine steering with weights
            steer_x = (self.sep_weight * sep_x +
                       self.align_weight * align_x +
                       self.coh_weight * coh_x +
                       self.boundary_weight * bx)
            steer_y = (self.sep_weight * sep_y +
                       self.align_weight * align_y +
                       self.coh_weight * coh_y +
                       self.boundary_weight * by)

            # If steering is tiny (isolated), add a small random wander to keep motion
            if abs(steer_x) < 1e-4 and abs(steer_y) < 1e-4:
                steer_x = math.cos(thetai) * 0.1 + random.uniform(-0.1, 0.1)
                steer_y = math.sin(thetai) * 0.1 + random.uniform(-0.1, 0.1)

            desired_theta = math.atan2(steer_y, steer_x)
            angle_diff = desired_theta - thetai
            # normalize
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            steer_mag = math.hypot(steer_x, steer_y)

            # Linear speed scales with how aligned we are (reduce speed if we need to turn) and steering strength
            linear = self.max_linear * (1.0 - min(1.0, abs(angle_diff) / math.pi)) * min(1.0, steer_mag / 2.0)
            linear = self._limit(linear, 0.02, self.max_linear)  # small min so they keep moving

            # Angular velocity: simple P controller on angle error
            angular = 2.0 * angle_diff
            angular = self._limit(angular, -self.max_angular, self.max_angular)

            # Publish Twist
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoidsSwarm(num_boids=6)  # change to spawn more/less boids (min 5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

