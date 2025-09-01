import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from turtlesim.msg import Pose


class circle_drawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        # Publisher to control the turtle
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # Timer to call publish_cmd every second
        self.timer = self.create_timer(1.0, self.publish_cmd)
        self.current_pose = None  # Variable to store the turtle's pose
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        ###############################
        self.radius = 2
        self.x_array = []
        self.y_array = []
        self.number_of_sides = 4

        center_x, center_y = 5.54, 5.54
        for i in range(self.number_of_sides + 1):  # Add 1 to return to the start
            # Use radians for math.cos/sin
            angle = i * 2 * math.pi / self.number_of_sides
            # The variable is self.radius, not self.r
            x = center_x+self.radius * math.cos(angle)
            y = center_y+self.radius * math.sin(angle)
            self.x_array.append(x)
            self.y_array.append(y)
        ###############################

        self.move_velocity = 0.6
        self.turn_velocity = 0.5

        self.target_index = 0

    def pose_callback(self, msg):
        """This function is called every time a new pose message is received."""
        self.current_pose = msg
        # self.get_logger().info(f'Turtle is at x: {self.current_pose.x:.2f}, y: {self.current_pose.y:.2f}')

    def publish_cmd(self):
        if self.current_pose is None:
            self.get_logger().info('Waiting for initial pose...')
            return

        msg = Twist()
        target_x = self.x_array[self.target_index]
        target_y = self.y_array[self.target_index]
        ####ANGLE CALCULATION####
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance_to_target = math.sqrt(dx ** 2 + dy ** 2)
        angle_to_target = math.atan2(dy, dx)
        #########################
        angle_to_rotate = angle_to_target - self.current_pose.theta
        #########################
        if angle_to_rotate > math.pi:
            angle_to_rotate -= 2 * math.pi
        elif angle_to_rotate < -math.pi:
            angle_to_rotate += 2 * math.pi

        if distance_to_target < 0.1:
            # Stop the turtle
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.target_index += 1

            if self.target_index >= len(self.x_array):
                self.target_index = 0
            self.get_logger().info(f'Moving to target #{self.target_index}')
        else:
            if abs(angle_to_rotate) > 0.1:
                msg.linear.x = 0.0
                msg.angular.z = self.turn_velocity * angle_to_rotate
            else:
                msg.linear.x = self.move_velocity * distance_to_target
                msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: forward command to next point')


def main(args=None):
    rclpy.init(args=args)
    node = circle_drawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
