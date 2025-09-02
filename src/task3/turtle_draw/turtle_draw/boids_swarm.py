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
        self.turtles = {}  # {name: {"pose": Pose, "pub": Publisher}}

        # Spawn turtles (besides turtle1)
        self.spawn_turtles()

        # Subscribe to all turtle poses + create publishers
        for i in range(1, self.num_turtles + 1):
            name = f"turtle{i}"
            self.turtles[name] = {
                "pose": None,
                "pub": self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            }
            self.create_subscription(Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), 10)

        # Control loop
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

        leader = self.turtles["turtle1"]["pose"]

        for name, data in self.turtles.items():
            pose = data["pose"]

            if name == "turtle1":  
                # Leader just goes forward
                cmd = Twist()
                cmd.linear.x = 1.0
                cmd.angular.z = 0.5
            else:
                # Followers chase leader
                angle_to_leader = math.atan2(leader.y - pose.y, leader.x - pose.x)
                angle_diff = math.atan2(math.sin(angle_to_leader - pose.theta),
                                        math.cos(angle_to_leader - pose.theta))

                cmd = Twist()
                cmd.linear.x = 1.0
                cmd.angular.z = angle_diff

            # --- Wall avoidance ---
            if pose.x < 1.0 or pose.x > 10.0 or pose.y < 1.0 or pose.y > 10.0:
                    target_angle += math.pi / 2.0  # turn away from wall
            
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
