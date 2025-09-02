import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class TurtleSpawner(Node):
    def __init__(self, num_turtles=5):
        super().__init__('turtle_spawner')
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        self.num_turtles = num_turtles
        self.spawn_turtles()

    def spawn_turtles(self):
        for i in range(1, self.num_turtles + 1):
            req = Spawn.Request()
            req.x = random.uniform(1.0, 10.0)
            req.y = random.uniform(1.0, 10.0)
            req.theta = random.uniform(0, 2*3.1415)
            req.name = f'turtle{i}'
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f'Spawned {future.result().name}')

def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    spawner.destroy_node()
    rclpy.shutdown()
