import math
import numpy
import random
import time
import rclpy
from turtlesim.msg import Pose
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

#i didn't have time to write a short letter, so i wrote a long one
# ~mark twain
# note from me: I swear i write better code, but it is literally 11:50 PM rn and i cannot, for the life of me,
# do anything anymore because my fingers feel so raw cuz the skin is peeling off(due to chem lab)
# to the person who is about to review this code, pls have a crocin, a dolo, and some tea ready. Good luck



class MinimalTurtleNode(Node):
    def __init__(self):
        super().__init__('minimal_turtle_node')
        self.xg=0
        self.yg=0
        self.numof_turtles = 5
        
        self.pos=[
            [0,0,0], 
            [0,0,0],
            [0,0,0],
            [0,0,0],
            [0,0,0],
            [0,0,0]
            ]




        # Publisher to control the turtle
        self.publisher_ = {}
        self.sp_spawn_and_publish(self.numof_turtles) #the n is how many turtles you want


        self.timer = self.create_timer(0.4, lambda: self.average())
        self.timer2 = self.create_timer(3, lambda: self.alignment())
        self.timer3 = self.create_timer(0.2, lambda: self.proximity())
        #self.timer4 = self.create_timer(0.3, lambda: self.wall())


        # gets the pose of the turtles
        i = 1
        while (i < self.numof_turtles+1):
            self.get_logger().info(f'{i}')
            command = f'/turtle{i}/pose'
            self.create_subscription(Pose, command, lambda msg, idx = i: self.find_pose(idx, msg), 10)
            i+=1


    #------ initializing commands
    def sp_spawn_and_publish(self, n):
        i = 1
        for i in range(1, n+1):
            self.make_turtle(f'turtle{i}')
            command = f'turtle{i}/cmd_vel'
            self.publisher_[f'turtle{i}'] = self.create_publisher(Twist, command, 10)

    def make_turtle(self, tname):
        self.cli = self.create_client(Spawn, 'spawn')
        req = Spawn.Request()
        req.x = random.randint(1, 9) * 1.0
        time.sleep(0.2)
        req.y = random.randint(1, 9) * 1.0
        req.theta = 0.0
        req.name = tname
    
        self.future=self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        
        self.get_logger().info(f"Turtle {tname} spawning")

        if self.future.result() is not None:
            self.get_logger().info(f"Turtle {tname} spawned")
        else:
            self.get_logger().error(f"Failed to spawn {tname}")
            self.make_turtle(tname)

    def find_pose(self, tn, msg: Pose):

        self.pos[tn][0]=msg.x
        self.pos[tn][1]=msg.y
        self.pos[tn][2]=msg.theta

        self.pos_logger(tn)


    def pos_logger(self, i):
        self.get_logger().info(f'[PL]--{i}-->{self.pos[i][0]}, {self.pos[i][1]}, {self.pos[i][2]}')

    def publish_cmd(self, distance, angle, tnum, deviation):

        msg = Twist()
        msg.linear.x = distance * 1.0   # move forward
        msg.angular.z = (angle + deviation) *1  # no rotation

        tname =f'turtle{tnum}'
        self.publisher_[tname].publish(msg)

#--------------
#--------------coherence
    def average(self):
        self.xg = 0
        self.yg = 0
        i = 0
        for i in range(self.numof_turtles):
            self.xg += self.pos[i+1][0]
            self.yg += self.pos[i+1][1]
            print(f'{i}, {self.numof_turtles}')

        self.xg = self.xg/self.numof_turtles
        self.yg = self.yg/self.numof_turtles

        i = 0
        for i in range(self.numof_turtles):
            self.goto(self.pos[i+1][0],self.pos[i+1][1], self.xg, self.yg, i+1)




    def goto(self, x, y, xg, yg, tnum):
        
        vals = self.distance(x,y,xg,yg)
        dist = vals[0]

        not_angle = math.atan2(vals[2],vals[1])
        angle1 = (((not_angle - self.pos[tnum][2]) + math.pi) % (2*math.pi) - math.pi)

        self.publish_cmd(dist, angle1, tnum, 0.0)


    def distance(self, x, y, xg, yg):
        xg = max(xg, 1)
        yg = max(yg, 1)
        xg = min(xg, 10.5)
        yg = min(yg, 10.5)

        X = xg - x
        Y = yg - y
        val=[None, None, None]
        val[0] = math.sqrt((X*X)+(Y*Y))
        val[1] = X
        val[2] = Y

        return val

#--------------
#--------------separation

    def proximity(self):
        for i in range(self.numof_turtles):
            print(i)
            n = i+1
            for n in range(self.numof_turtles):
                if ((int(self.pos[i+1][0]) in range(int(self.pos[n][0]),int(self.pos[n][0]+0.2)))):
                    #&(int(self.pos[i][1]) in range(int(self.pos[n][1]),int(self.pos[n][1]+0.2)))):
                    self.get_logger().info(f'obstacle detected, i={i+1}, n={n}')
                    print(i+1)
                    time.sleep(1)
                    self.publish_cmd(1.7, (0.1*math.pi)*1.0, i+1, 0.0)
                    


#--------------
#--------------alignment
    def alignment(self):
        avg_angle = 0
        for i in range(self.numof_turtles):
            avg_angle +=self.pos[i+1][2]
        avg_angle /= self.numof_turtles
        for i in range(self.numof_turtles):
            self.publish_cmd(2, avg_angle-self.pos[i+1][2], i+1, 0) 




#---------- wall avoiding ----- nvm this sucks, let them collide
'''
    def wall(self):
        x_correction = 0
        y_correction = 0
        strength = 1

        for i in range(self.numof_turtles):
            if (self.pos[i+1][0] > 9.5):
                x_correction -= strength
            if (self.pos[i+1][0] < 1.5):
                x_correction += strength

            if (self.pos[i+1][1] > 9.5):
                y_correction -= strength
            if (self.pos[i+1][1] < 1.5):
                y_correction += strength
            self.goto(self.pos[i+1][0], self.pos[i+1][1], x_correction, y_correction, i+1)
            x_correction = 0
            y_correction = 0

            #why do some turtles pass through the barrier but the others cant???!?!?!?
'''



                





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
