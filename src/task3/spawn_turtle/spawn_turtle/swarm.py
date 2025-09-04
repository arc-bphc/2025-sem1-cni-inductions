import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import random
import math


class TurtleSwarm(Node):
    def __init__(self):
        super().__init__("turtle_swarm")

        self.num_turtles = 5
        self.turtle_names = ["turtle1"]

        # Spawn extra turtles
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            pass  # wait for service

        for i in range(2, self.num_turtles + 1):
            req = Spawn.Request()
            req.x = random.uniform(2.0, 9.0)  
            req.y = random.uniform(2.0, 9.0)
            req.theta = random.uniform(0, 2 * math.pi)
            req.name = f"turtle{i}"
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.turtle_names.append(future.result().name)

       
        self.pose_data = {}
        self.cmd_publishers = {}
        self.last_vx = {}
        self.last_vy = {}

        for name in self.turtle_names:
            self.pose_data[name] = None
            self.cmd_publishers[name] = self.create_publisher(
                Twist, f"/{name}/cmd_vel", 10
            )
            self.create_subscription(
                Pose, f"/{name}/pose", self.make_pose_callback(name), 10
            )

        # Timer to update swarm
        self.timer = self.create_timer(0.1, self.update_swarm)  

    def make_pose_callback(self, name):
        def callback(msg):
            self.pose_data[name] = msg
        return callback

    def update_swarm(self):
        if not all(self.pose_data.values()):
            return  # wait until all poses are received

        for name in self.turtle_names:
            my_pose = self.pose_data[name]
            neighbors = [self.pose_data[n] for n in self.turtle_names if n != name]
            if not neighbors:
                continue

            # --- Cohesion ---
            avg_x = sum(n.x for n in neighbors) / len(neighbors)
            avg_y = sum(n.y for n in neighbors) / len(neighbors)
            cohesion_x = avg_x - my_pose.x
            cohesion_y = avg_y - my_pose.y

            # --- Separation (turtle-to-turtle) ---
            sep_x, sep_y = 0.0, 0.0
            for n in neighbors:
                dx = my_pose.x - n.x
                dy = my_pose.y - n.y
                dist = math.sqrt(dx**2 + dy**2)
                if 0.0 < dist < 3.0:  # collision 
                    strength = (3.0 - dist) / 3.0
                    sep_x += dx / dist * strength
                    sep_y += dy / dist * strength

            # --- Alignment ---
            avg_theta = sum(n.theta for n in neighbors) / len(neighbors)
            align_theta = avg_theta - my_pose.theta
            align_theta = math.atan2(math.sin(align_theta), math.cos(align_theta))

            # --- Wall avoidance ---
            wall_x, wall_y = 0.0, 0.0
            margin = 2.0  # how close before repelling
            strength = 1.0
            if my_pose.x < margin:  # left wall
                wall_x += strength * (margin - my_pose.x)
            if my_pose.x > 11.0 - margin:  # right wall
                wall_x -= strength * (my_pose.x - (11.0 - margin))
            if my_pose.y < margin:  # bottom wall
                wall_y += strength * (margin - my_pose.y)
            if my_pose.y > 11.0 - margin:  # top wall
                wall_y -= strength * (my_pose.y - (11.0 - margin))

            # --- Wander term ---
            wx = random.uniform(-2.0, 2.0)
            wy = random.uniform(-2.0, 2.0)

        
            vx = 0.005 * cohesion_x + 0.3 * sep_x + 0.2 * wx + 0.5 * wall_x
            vy = 0.005 * cohesion_y + 0.3 * sep_y + 0.2 * wy + 0.5 * wall_y

            # Optional smoothing
            alpha = 0.7
            self.last_vx[name] = alpha * self.last_vx.get(name, 0.0) + (1 - alpha) * vx
            self.last_vy[name] = alpha * self.last_vy.get(name, 0.0) + (1 - alpha) * vy
            vx, vy = self.last_vx[name], self.last_vy[name]

            target_angle = math.atan2(vy, vx)
            angle_error = target_angle - my_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            msg = Twist()
            msg.linear.x = 1.0   # constant forward speed
            msg.angular.z = 2.0 * angle_error + 0.1 * align_theta

            self.cmd_publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSwarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
