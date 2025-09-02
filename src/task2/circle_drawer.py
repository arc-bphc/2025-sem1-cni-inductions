# base code borrowed from example-submissions

import math
import random
import time
import rclpy
from turtlesim.msg import Pose
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


'''
    basic idea:
    1) get radius
    2) x = radius*cos(bp_t*pi), y = r*sin(bp_t*pi) <--- first one is init
    2.5)go to x,y
    3) bp_t+=0.01
    4) calculate x' and y'
    5) go to x',y'x
    --- continue till you reach init
    at the end: goto init
    exit

'''



class MinimalTurtleNode(Node):
    def __init__(self):
        radius = float(input("radius: "))
        super().__init__('minimal_turtle_node')
        # Publisher to control the turtle
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Timer to call publish_cmd every second
        self.timer = self.create_timer(0.5, lambda: self.find_pt(radius))

        self.pose = None
        self.x = None
        self.y = None
        self.theta = None
        self.xcen = None
        self.ycen = None
        self.bp_theta = float(0.0)
        self.i = 0

        self.create_subscription(Pose, '/turtle1/pose', self.find_pose, 10)

    def find_pose(self, msg: Pose):
        self.pose = msg
        if (self.xcen == None):  
            self.xcen = msg.x
            self.ycen = msg.y

        self.x=msg.x
        self.y=msg.y
        self.theta=msg.theta

    def find_pt(self, radius):
        theta = self.bp_theta
        cosval = math.cos(theta)
        sinval = math.sin(theta)
        x = self.xcen + (radius * cosval)
        y = self.ycen + (radius * sinval)
        self.bp_theta += 0.01
        self.goto(x, y, radius)

        
        


    def goto(self, xg, yg, radius):

        X = self.x - xg
        Y = self.y - yg
        dist = math.sqrt((X*X)+(Y*Y))
        not_angle = math.atan2(Y,X)
        angle = ((not_angle) + math.pi) % (2*math.pi) - math.pi

        self.publish_cmd(dist, angle, radius)




    def publish_cmd(self, distance, angle, radius):

        msg = Twist()
        msg.linear.x = 1.0 * radius 
        msg.angular.z = 2.0/radius  
        self.publisher_.publish(msg)

        if(math.isclose(self.x, self.xcen, abs_tol=0.5))&(math.isclose(self.y, self.ycen, abs_tol=0.5))&(self.i >= 1):
            self.get_logger().info("reached initial point")
            msg.linear.x = 0.0
            msg.angular.z=0.0
            self.publisher_.publish(msg)
            exit(1)
        else:
            self.i=1





def main(args=None):
    rclpy.init(args=args)
    node = MinimalTurtleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#damn :)