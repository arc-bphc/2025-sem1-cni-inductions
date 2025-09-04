import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class Mightynode(Node):
    def __init__(self):
        super().__init__('spawn_turtle_node')  
        self.turtle_speeds = []
        self.turtle_angs = []
        self.turtle_publishers = []
        self.turtle_poses = {}
        self.client = self.create_client(Spawn, 'spawn')
        self.v_comps = []

    def spawn(self,name):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        req = Spawn.Request()
        req.x = random.uniform(1.0, 10.0)        
        req.y = random.uniform(1.0, 10.0)    
        req.theta = random.uniform(0.0, 2*math.pi)
        req.name = name
        future = self.client.call_async(req)
        # self.turtle_angs.append(req.theta)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {future.result().name}")
        else:
            self.get_logger().error("Service call failed!")
    
    def move_turtles(self,pub):
        msg = Twist()
        msg.linear.x = random.uniform(3,6)
        msg.angular.z = 0.0
        pub.publish(msg)
        self.get_logger().info(f"Moving with speed {msg.linear.x:.2f}")
        # self.turtle_speeds.append(msg.linear.x)
        for _,pose in self.turtle_poses.items():
            speed = msg.linear.x
            theta = pose.theta   
            self.turtle_speeds.append(speed)
            self.turtle_angs.append(theta)

    def update_velocity(self,steer,steer_angle,pub):
        k=2
        msg = Twist()
        msg.linear.x = np.linalg.norm(steer+3) 
        msg.angular.z = k * steer_angle
        pub.publish(msg)

    def Alignment(self,turtle_velocity,turtle_angle):
        avg_velocity = sum(self.v_comps)/len(self.v_comps)
        steer = avg_velocity - turtle_velocity
        desired_angle = math.atan2(steer[1], steer[0])
        steer_angle = (desired_angle - turtle_angle + math.pi) % (2*math.pi) - math.pi
        self.turtle_speeds.append(np.linalg.norm(steer))
        self.turtle_angs.append(steer_angle)
        return steer,steer_angle


    def timer_callback(self):     
        self.v_comps.clear()   
        self.v_comps = [
        np.array([self.turtle_speeds[i] * math.cos(self.turtle_angs[i]),
                self.turtle_speeds[i] * math.sin(self.turtle_angs[i])])
        for i in range(5)
        ]
        dup = self.turtle_angs.copy()
        self.turtle_speeds.clear()
        self.turtle_angs.clear()
        for i,pub in enumerate(self.turtle_publishers):
            self.update_velocity(*self.Alignment(self.v_comps[i],dup[i]),pub)


        # self.turtle_speeds.clear()
        # self.turtle_angs.clear()
        # self.v_comps.clear()


    def pose_callback(self,turtle_name,msg):
        self.turtle_poses[turtle_name] = msg
        
def main(args=None):
    names = []
    rclpy.init(args=args)           
    node = Mightynode()

    for i in range(5):
        # Creating Publisher channels for each turtle
        name = f'turtle{i+2}'
        names.append(name)
        pub_name = node.create_publisher(Twist, f'{name}/cmd_vel', 10)
        node.turtle_publishers.append(pub_name)
        node.move_turtles(node.turtle_publishers[i])
        
        #Subscription for Pose messages - i finally understood subscriptionss
        node.create_subscription(Pose,f'{name}/pose',lambda msg, n=name: node.pose_callback(n, msg),10)
        node.spawn(name)
        # node.v_comps.append()
    node.timer = node.create_timer(0.1, node.timer_callback)
    rclpy.spin(node)   
    node.destroy_node()             
    rclpy.shutdown()              

if __name__ == '__main__':
    main()