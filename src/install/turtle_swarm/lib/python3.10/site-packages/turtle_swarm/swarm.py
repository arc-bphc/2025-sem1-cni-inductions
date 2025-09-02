#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import math
import random
import time

def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d

class SwarmNode(Node):
    def __init__(self):
        super().__init__('swarm_node')

        # Parameters (tweak these)
        self.declare_parameter('num_turtles', 5)
        self.declare_parameter('neighbor_radius', 2.0)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_angular', 4.0)
        self.declare_parameter('w_sep', 1.6)
        self.declare_parameter('w_align', 1.0)
        self.declare_parameter('w_cohesion', 1.0)
        self.declare_parameter('wall_push', 1.5)

        self.N = self.get_parameter('num_turtles').value
        self.neighbor_radius = self.get_parameter('neighbor_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular = self.get_parameter('max_angular').value
        self.w_sep = self.get_parameter('w_sep').value
        self.w_align = self.get_parameter('w_align').value
        self.w_cohesion = self.get_parameter('w_cohesion').value
        self.wall_push = self.get_parameter('wall_push').value

        # storage
        self.turtles = []                # list of kachua names (strings)
        self.poses = {}                  # name -> latest Position
        self.pub = {}                    # name -> publisher to /turtleX/cmd_vel (FIXED) if i called it publisehr problem was happening

        # spawn kachua
        self.get_logger().info(f"Spawning {self.N-1} additional turtles (total {self.N})")
        self.spawn_turtles()

        # timer for loop loop (10 oer second)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Swarm node ready.")

    def spawn_turtles(self):
        client = self.create_client(Spawn, 'spawn')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Spawn service not available; make sure turtlesim_node is running.')
            return

        for i in range(1, self.N+1):
            name = f'turtle{i}'
            if i == 1:
                self.register_turtle(name)
                continue

            x = random.uniform(1.0, 10.0)
            y = random.uniform(1.0, 10.0)
            theta = random.uniform(-math.pi, math.pi)

            req = Spawn.Request()
            req.x = float(x)
            req.y = float(y)
            req.theta = float(theta)
            req.name = name

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                resp = future.result()
                new_name = resp.name
                self.get_logger().info(f"Spawned {new_name} at ({x:.2f}, {y:.2f})")
                self.register_turtle(new_name)
            else:
                self.get_logger().error(f"Failed to spawn {name}")

    def register_turtle(self, name):
        pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)

        def make_cb(n):
            def cb(msg):
                self.poses[n] = msg
            return cb

        sub = self.create_subscription(Pose, f'/{name}/pose', make_cb(name), 10)

        self.turtles.append(name)
        self.pub[name] = pub              # FIXED
        self.poses[name] = None

    def control_loop(self):
        available = [n for n in self.turtles if self.poses[n] is not None]
        if len(available) < 1:
            return

        info = {}
        for n in available:
            p = self.poses[n]
            info[n] = (p.x, p.y, p.theta, p.linear_velocity)

        for name in available:
            x, y, theta, linv = info[name]

            neighbors = []
            for other in available:
                if other == name: continue
                ox, oy, otheta, olinv = info[other][0], info[other][1], info[other][2], info[other][3]
                dist = math.hypot(ox - x, oy - y)
                if dist <= self.neighbor_radius:
                    neighbors.append((other, ox, oy, otheta, olinv, dist))

            if not neighbors:
                tmsg = Twist()
                tmsg.linear.x = max(0.5, self.max_speed * 0.6)
                tmsg.angular.z = random.uniform(-0.5, 0.5)
                self.pub[name].publish(tmsg)   # FIXED
                continue

            # Boids rules
            sep_x = sep_y = 0.0
            align_x = align_y = 0.0
            com_x = com_y = 0.0

            for (_, ox, oy, otheta, _, dist) in neighbors:
                if dist != 0:
                    dx = x - ox
                    dy = y - oy
                    inv = 1.0 / (dist + 0.01)
                    sep_x += dx * inv
                    sep_y += dy * inv
                align_x += math.cos(otheta)
                align_y += math.sin(otheta)
                com_x += ox
                com_y += oy

            align_x /= max(1, len(neighbors))
            align_y /= max(1, len(neighbors))
            com_x /= max(1, len(neighbors))
            com_y /= max(1, len(neighbors))
            coh_x = com_x - x
            coh_y = com_y - y

            vx = self.w_sep * sep_x + self.w_align * align_x + self.w_cohesion * coh_x
            vy = self.w_sep * sep_y + self.w_align * align_y + self.w_cohesion * coh_y

            if x < 1.0: vx += self.wall_push*(1.0 - x)
            if x > 10.0: vx -= self.wall_push*(x - 10.0)
            if y < 1.0: vy += self.wall_push*(1.0 - y)
            if y > 10.0: vy -= self.wall_push*(y - 10.0)

            desired_angle = math.atan2(vy, vx)
            desired_speed = min(math.hypot(vx, vy), self.max_speed)

            err = angle_diff(desired_angle, theta)

            K_ang = 4.0
            ang_cmd = max(-self.max_angular, min(self.max_angular, K_ang*err))
            lin_cmd = desired_speed * max(0.0, math.cos(err))
            if lin_cmd < 0.2 and abs(err) < 0.3:
                lin_cmd = min(0.2, desired_speed)

            tmsg = Twist()
            tmsg.linear.x = float(lin_cmd)
            tmsg.angular.z = float(ang_cmd)
            self.pub[name].publish(tmsg)       # FIXED

def main(args=None):
    rclpy.init(args=args)
    node = SwarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
