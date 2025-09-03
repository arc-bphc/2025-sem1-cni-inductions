import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import random

class SwarmTurtle():
    def __init__(self, name, swarm):
        self.name = name
        self.sub = swarm.create_subscription(Pose, f'/{self.name}/pose', self.pose_cb, 10)
        self.pub = swarm.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.swarm = swarm
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def pose_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def update(self):
        self.swarm.get_logger().info(f"x: {self.x}")
        pass

class SwarmNode(Node):
    def __init__(self, num_turtles):
        super().__init__('swarm_node')
        self.spawn_cli = self.create_client(Spawn, 'spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        clear_cli = self.create_client(Empty, '/reset')
        while not clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        clear_cli.call_async(Empty.Request())
 

        self.turtles = [SwarmTurtle("turtle1", self)]
        self.spawn_swarm(num_turtles)
        self.timer = self.create_timer(0.01, self.update)

    def spawn_swarm(self, num_turtles):
       for i in range(2, num_turtles+1):
            self.spawn_request(i)
            swarm_turtle = SwarmTurtle("turtle" + str(i), self)
            self.turtles.append(swarm_turtle)
 
    def spawn_request(self, num):
        req = Spawn.Request()
        req.x = random.uniform(1.0, 10.5)
        req.y = random.uniform(1.0, 10.5)
        req.theta = random.uniform(0.0, 360.0)
        req.name = "turtle" + str(num)
        self.spawn_cli.call_async(req)

    def update(self):
        for turtle in self.turtles:
            turtle.update()


def main(args=None):
    rclpy.init(args=args)
    num_turtles = 5
    swarm_node = SwarmNode(num_turtles)
    try:
        rclpy.spin(swarm_node)
    except KeyboardInterrupt:
        pass

    swarm_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
