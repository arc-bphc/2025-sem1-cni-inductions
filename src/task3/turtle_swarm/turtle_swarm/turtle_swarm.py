import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import random

NUM_TURTLES = 5
NEIGHBOR_RADIUS = 4.0   
SEP_RADIUS = 2.0
V_MAX = 1.5

W_SEP = 4.0
W_ALIGN = 1.0
W_COH = 1.0


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class BoidTurtle(Node):
    def __init__(self):
        super().__init__('boid_turtle_controller')

        # Store positions and velocities
        self.poses = {}

        # Random initial velocities so turtles start moving
        self.velocities = {
            f'turtle{i}': (random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0))
            for i in range(1, NUM_TURTLES + 1)
        }

        # Publishers for each turtle (renamed to avoid clash with rclpy internals)
        self.cmd_publishers = {}
        for i in range(1, NUM_TURTLES + 1):
            name = f'turtle{i}'
            self.cmd_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), 10)

        # Update loop at 10 Hz
        self.timer = self.create_timer(0.1, self.update)

    def pose_callback(self, msg, name):
        self.poses[name] = msg

    def update(self):
        for name, pose in self.poses.items():
            # Compute flocking velocity
            vx, vy = self.compute_boid_velocity(name)

            # Convert to speed + heading
            angle_to_goal = math.atan2(vy, vx)
            speed = min(math.hypot(vx, vy), V_MAX)

            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = normalize_angle(angle_to_goal - pose.theta)
            self.cmd_publishers[name].publish(twist)

            # Store velocity for alignment next time
            self.velocities[name] = (vx, vy)

            # Debug logging
            self.get_logger().info(f"{name}: vx={vx:.2f}, vy={vy:.2f}, speed={speed:.2f}")

    def compute_boid_velocity(self, name):
        pos_i = self.poses[name]
        vx_i, vy_i = self.velocities[name]

        sep = [0.0, 0.0]
        align = [0.0, 0.0]
        coh = [0.0, 0.0]

        count_align = 0
        count_coh = 0

        for other_name, other_pose in self.poses.items():
            if other_name == name:
                continue

            dx = other_pose.x - pos_i.x
            dy = other_pose.y - pos_i.y
            dist = math.hypot(dx, dy)
            if dist == 0:
                continue

            # ----- Separation (UNNORMALIZED) -----
            if dist < SEP_RADIUS:
                # Unit away vector
                ax = -dx / dist
                ay = -dy / dist

                # Closeness weighting: 0..1 (stronger when closer)
                strength = (SEP_RADIUS - dist) / SEP_RADIUS

                # Extra push if on a collision course (relative approach speed)
                vx_j, vy_j = self.velocities.get(other_name, (0.0, 0.0))
                rvx = vx_i - vx_j
                rvy = vy_i - vy_j
                approach = max(0.0, (dx * rvx + dy * rvy) / (dist + 1e-6))  # >0 if moving toward each other

                # Short-range collision bubble (< 0.5m): very strong
                bubble = 0.0
                if dist < 0.5:
                    bubble = 4.0 * (0.5 - dist) / 0.5

                sep[0] += (ax * (1.0 + 2.0 * approach) * strength + ax * bubble) * W_SEP
                sep[1] += (ay * (1.0 + 2.0 * approach) * strength + ay * bubble) * W_SEP

            # ----- Alignment & Cohesion (normalized later) -----
            if dist < NEIGHBOR_RADIUS:
                vx_j, vy_j = self.velocities.get(other_name, (0.0, 0.0))
                align[0] += vx_j
                align[1] += vy_j
                count_align += 1

                coh[0] += other_pose.x
                coh[1] += other_pose.y
                count_coh += 1

        # Process alignment: average -> unit -> scaled to V_MAX -> weighted
        if count_align > 0:
            align[0] /= count_align
            align[1] /= count_align
            mag = math.hypot(align[0], align[1])
            if mag > 0:
                align[0] = (align[0] / mag) * V_MAX * W_ALIGN
                align[1] = (align[1] / mag) * V_MAX * W_ALIGN

        # Process cohesion: vector to neighbors’ centroid -> unit -> scaled -> weighted
        if count_coh > 0:
            coh[0] = (coh[0] / count_coh) - pos_i.x
            coh[1] = (coh[1] / count_coh) - pos_i.y
            mag = math.hypot(coh[0], coh[1])
            if mag > 0:
                coh[0] = (coh[0] / mag) * V_MAX * W_COH
                coh[1] = (coh[1] / mag) * V_MAX * W_COH

        # ----- Desired velocity from rules -----
        vx_desired = sep[0] + align[0] + coh[0]
        vy_desired = sep[1] + align[1] + coh[1]

        # ----- Boundary force (smooth ramp) -----
        MARGIN = 2.0
        ARENA = 11.0
        WALL = 3.0  # wall strength

        if pos_i.x < MARGIN:
            vx_desired += WALL * (MARGIN - pos_i.x) ** 2
        elif pos_i.x > ARENA - MARGIN:
            vx_desired -= WALL * (pos_i.x - (ARENA - MARGIN)) ** 2

        if pos_i.y < MARGIN:
            vy_desired += WALL * (MARGIN - pos_i.y) ** 2
        elif pos_i.y > ARENA - MARGIN:
            vy_desired -= WALL * (pos_i.y - (ARENA - MARGIN)) ** 2

        # Clip desired to feasible speed before steering
        des_speed = math.hypot(vx_desired, vy_desired)
        if des_speed > V_MAX:
            vx_desired = (vx_desired / des_speed) * V_MAX
            vy_desired = (vy_desired / des_speed) * V_MAX

        # ----- Steering (rate-limited acceleration) -----
        vx_cur, vy_cur = self.velocities[name]
        steer_x = vx_desired - vx_cur
        steer_y = vy_desired - vy_cur

        MAX_FORCE = 1.2
        mag = math.hypot(steer_x, steer_y)
        if mag > MAX_FORCE:
            steer_x = (steer_x / mag) * MAX_FORCE
            steer_y = (steer_y / mag) * MAX_FORCE

        vx = vx_cur + steer_x
        vy = vy_cur + steer_y

        # Final speed clamp
        speed = math.hypot(vx, vy)
        if speed > V_MAX:
            vx = (vx / speed) * V_MAX
            vy = (vy / speed) * V_MAX

        return vx, vy



def main(args=None):
    rclpy.init(args=args)
    node = BoidTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
