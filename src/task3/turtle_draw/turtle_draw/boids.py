import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import math
from geometry_msgs.msg import Twist
class SpawnTurtleNode(Node):
    def __init__(self):
        super().__init__('spawn_turtle_node')  
        self.turtle_publishers = []

        self.client = self.create_client(Spawn, 'spawn')
    
    def spawn(self,name):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        req = Spawn.Request()
        req.x = random.uniform(1.0, 10.0)        
        req.y = random.uniform(1.0, 10.0)    
        req.theta = random.uniform(0.0, 2*math.pi)
        req.name = name
        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {future.result().name}")
        else:
            self.get_logger().error("Service call failed!")
    
    def move_turtles(self,pub):
        msg = Twist()
        msg.linear.x = random.uniform(0.5, 3.0)
        msg.angular.z = 0.0
        pub.publish(msg)
        self.get_logger().info(f"Moving with speed {msg.linear.x:.2f}")
    
    def timer_callback(self):
        for pub in self.turtle_publishers:
            self.move_turtles(pub)


def main(args=None):
    names = []
    rclpy.init(args=args)           
    node = SpawnTurtleNode()

    for i in range(5):
        name = f'turtle{i+2}'
        names.append(name)
        pub_name = node.create_publisher(Twist, f'{name}/cmd_vel', 10)
        node.turtle_publishers.append(pub_name)
        node.spawn(name)
    node.timer = node.create_timer(0.5, node.timer_callback)
    rclpy.spin(node)   
    node.destroy_node()             
    rclpy.shutdown()               

if __name__ == '__main__':
    main()