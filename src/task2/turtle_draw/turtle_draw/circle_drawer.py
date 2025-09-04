import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class TurtleCirclePoints(Node):
    def __init__(self):
        super().__init__("turtle_circle_points")

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Subscriber to get the turtle's pose
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.r = 2.0
        self.num_points = 36

        self.points = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = 5.5 + self.r * math.cos(theta)  # shift circle center to (5.5, 5.5)
            y = 5.5 + self.r * math.sin(theta)
            self.points.append((x, y))

        self.current_index = 0
        self.pose = None 

    def pose_callback(self, msg: Pose):
        """Callback to update turtle's current pose."""
        self.pose = msg

    def timer_callback(self):
        if self.pose is None:
            return  

        # Get current target point
        target_x, target_y = self.points[self.current_index]

        # Compute distance and angle to target
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        target_theta = math.atan2(dy, dx)

        # Control law
        msg = Twist()
        if distance > 0.1:  # keep moving until close enough
            # Angular control: turn towards target
            angle_error = target_theta - self.pose.theta
            # Normalize angle to [-pi, pi]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            msg.linear.x = 2.0 * distance        
            msg.angular.z = 6.0 * angle_error   
        else:
            # Reached point → switch to next
            self.current_index = (self.current_index + 1) % self.num_points

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCirclePoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
