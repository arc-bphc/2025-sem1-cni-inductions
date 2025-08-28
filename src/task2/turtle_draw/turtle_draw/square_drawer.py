"""
 *  author: realpratz [Pratyush Priyadarshi]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
#from turtlesim.msg import Pose
import math

class square_drawer(Node):
    def __init__(self):
        super().__init__('square_drawer')
        
        self.n = 4 #---n=square is of 4 sides. this is just used for array size mainly
        self.t = 0.1 #---timer period
        self.r = 2.0 #---length of said square
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
        r=self.r
        #---just listing out coordinates in distance, we're using (r,r) two times, this is to send it back to where it began for it to complete it's 4th vertex (clearly, edges=vertices-1)
        p = [[r,r],[-r,r],[-r,-r],[r,-r],[r,r]]
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
    node = square_drawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()