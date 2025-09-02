import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_equation_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)

        
        self.radius = 2.0
        self.center_x = 5.5   # Center of screen (turtlesim window is 11x11) in noVNC its 5.5 idk about the other ones
        self.center_y = 5.5
        self.num_points = 100
        self.current_point = 0

        # generating points....
        self.points = []
        for i in range(self.num_points):
            theta = (2 * math.pi / self.num_points) * i
            x = self.center_x + self.radius * math.cos(theta)
            y = self.center_y + self.radius * math.sin(theta)
            self.points.append((x, y))

        self.pose = None
        self.timer = self.create_timer(0.1, self.move_turtle)

    def update_pose(self, msg):
        self.pose = msg

    def move_turtle(self):
        if self.pose is None:
            return

        goal_x, goal_y = self.points[self.current_point]
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y

        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        # Fixing the error to be within -pi to pi
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        msg = Twist()

        if distance > 0.1:  
            msg.linear.x = 1.0 * distance       
            msg.angular.z = 4.0 * angle_error   
        else:
            
            self.current_point = (self.current_point + 1) % self.num_points

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
