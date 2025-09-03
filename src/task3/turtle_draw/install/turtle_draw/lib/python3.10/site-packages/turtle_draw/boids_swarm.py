import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
import math


class SwarmNode(Node):
    def __init__(self):
        super().__init__('swarm_node')

        self.num_turtles = 5
        self.turtles = {} 

        self.spawn_turtles()

        for i in range(1, self.num_turtles + 1):
            name = f"turtle{i}"
            self.turtles[name] = {
                "pose": None,
                "pub": self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            }
            self.create_subscription(Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), 10)

        self.timer = self.create_timer(0.1, self.update_swarm)

    def spawn_turtles(self):
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        for i in range(2, self.num_turtles + 1):
            req = Spawn.Request()
            req.x = random.uniform(2.0, 8.0)
            req.y = random.uniform(2.0, 8.0)
            req.theta = random.uniform(-math.pi, math.pi)
            req.name = f"turtle{i}"

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"Spawned {req.name}")
            else:
                self.get_logger().error(f"Failed to spawn {req.name}")


    def pose_callback(self, msg: Pose, name: str):
        self.turtles[name]["pose"] = msg

            
    def update_swarm(self):
        if not all(self.turtles[t]["pose"] for t in self.turtles):
            return  

        WORLD_MIN, WORLD_MAX = 0.0, 11.0
        CENTER_X, CENTER_Y = (WORLD_MIN + WORLD_MAX) / 2.0, (WORLD_MIN + WORLD_MAX) / 2.0
        SAFE_MARGIN = 1.5

        for name, data in self.turtles.items():
            pose = data["pose"]
            neighbors = [self.turtles[n]["pose"] for n in self.turtles if n != name]

            cmd = Twist()

            if not neighbors:
                continue

            # boids rules
            # cohesion
            avg_x = sum(n.x for n in neighbors) / len(neighbors)
            avg_y = sum(n.y for n in neighbors) / len(neighbors)
            cohesion_angle = math.atan2(avg_y - pose.y, avg_x - pose.x)

            #separation
            sep_x, sep_y = 0.0, 0.0
            for n in neighbors:
                dx, dy = pose.x - n.x, pose.y - n.y
                dist = math.hypot(dx, dy)
                if dist < 1.0:
                    cmd.linear.x = 1.0
                    sep_x += dx / (dist + 1e-6)
                    sep_y += dy / (dist + 1e-6)
                elif dist < 2:
                    cmd.linear.x = 1.5 #speed up if farther away
                else:
                    cmd.linear.x = 2.0 #speed up even more if too far
            separation_angle = math.atan2(sep_y, sep_x) if (sep_x or sep_y) else pose.theta

            # alignment
            avg_theta = sum(n.theta for n in neighbors) / len(neighbors)

            boids_target = (
                0.5 * cohesion_angle +
                0.2 * separation_angle +
                0.3 * avg_theta
            )

            #wall avoidance
            near_wall = (
                pose.x < SAFE_MARGIN or pose.x > WORLD_MAX - SAFE_MARGIN or
                pose.y < SAFE_MARGIN or pose.y > WORLD_MAX - SAFE_MARGIN
            )

            if near_wall:
                target_angle = math.atan2(CENTER_Y - pose.y, CENTER_X - pose.x)
            else:
                target_angle = boids_target

            #smooth steering
            angle_diff = math.atan2(
                math.sin(target_angle - pose.theta),
                math.cos(target_angle - pose.theta)
            )

            
            cmd.angular.z = angle_diff
            data["pub"].publish(cmd)





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
