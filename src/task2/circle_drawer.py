import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        
       
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
       
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.pose = None  
        self.radius = 2.0
        self.num_points = 36  
        self.points = self.generate_points()
        self.point_index = 0

        
        time.sleep(1.0)  
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        self.pose = msg

    def generate_points(self):
        points = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = self.radius * math.cos(theta) + 5.5   
            points.append((x, y))
        return points

    def timer_callback(self):
        if self.pose is None:
            return
        
       
        goal_x, goal_y = self.points[self.point_index]
        
       
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta
        
        
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        msg = Twist()
        
        if distance > 0.1: 
            msg.linear.x = 1.5 * distance
            msg.angular.z = 4.0 * angle_error
        else: 
            self.point_index = (self.point_index + 1) % self.num_points
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()