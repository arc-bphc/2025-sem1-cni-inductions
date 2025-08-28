"""
 *  author: realpratz [Pratyush Priyadarshi]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
#from turtlesim.msg import Pose
import math

class star_drawer(Node):
    def __init__(self):
        super().__init__('star_drawer')
        
        self.n = 5 #---n=star is of 5 sides. this is just used for array size mainly
        self.t = 0.1 #---timer period
        self.r = 2.0 #---inner radius of star
        #self.theta = 0.0 #---where turtle facing
        self.points = self.gen_p() #---generation of points
        self.cur_i = 0 #---current index out of generated points

        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10) #---Publisher to control the turtle
        #self.subcriber_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10) #---Subscriber to get back turtle data
        self.timer = self.create_timer(self.t, self.publish_cmd) #---Timer to call publish_cmd every t sec

    """
    def pose_callback(self, msg):
        self.position = [msg.x, msg.y]
        self.theta = msg.theta
    """

    def gen_p(self):
        p = []
        verts = []

        #---the wiki says that "The pentagram can be constructed by connecting alternate vertices of a pentagon", so we'll generate 5 points with 72 degree increments, but order them as 0,2,4,1,3 and finish with 0 again.
        for i in range(5):
            angle = 2 * math.pi * i / 5 
            x = self.r * math.cos(angle)
            y = self.r * math.sin(angle)
            verts.append([x, y])

        order = [0, 2, 4, 1, 3, 0]
        for i in order:
            p.append(verts[i])

        return p

    def stop(self):
        msg = Twist()
        msg.linear.x = msg.linear.y = msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def publish_cmd(self):
        if(self.cur_i >= self.n): 
            self.stop()
            raise SystemExit

        x1, y1 = self.points[self.cur_i]
        x2, y2 = self.points[self.cur_i + 1]

        msg = Twist()

        """
        # where I should be pointing if I align with my movement direction
        desired_theta = math.atan2(msg.linear.y, msg.linear.x) 

        # how much the boid needs to rotate to face the velocity vector
        angle_diff = desired_theta - self.theta

        # we want the shortest signed angle diff (beyond -pi and pi, it'll repeat)
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi #(if t>pi, then subtracting 2pi makes it between [-pi,pi])
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi #(if t<pi, then adding 2pi makes it between [-pi,pi])

        msg.angular.z = angle_diff * 2.0
        """

        msg.angular.z = 0.0

        #---v=s/t; s=distance between points, t is timer period, both dimensions are independent of each other (vectors can be decomposed into components and vice-versa), so calculate them independently
        msg.linear.x = (x2-x1)/self.t
        msg.linear.y = (y2-y1)/self.t

        self.cur_i+=1
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = star_drawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()