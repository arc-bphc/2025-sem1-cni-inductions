import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi, sin, cos, floor

class CircleTurtleNode(Node):
    def __init__(self):
        super().__init__('circle_turtle_node')

        self.frequency = 0.1

        # circle parameters
        self.circle_radius = 2

        # initializing points list
        self.points = []
        for theta in range(0, 2*floor(pi*10)+1):
            theta /= 10
            self.points.append((self.circle_radius*cos(theta), self.circle_radius*sin(theta)))
        self.cur_point_index = 0

        # publisher
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # timer
        self.timer = self.create_timer(self.frequency, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        if self.cur_point_index == len(self.points)-1:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info("done drawing")
            raise SystemExit
            return

        # displacement calculation
        current_point = self.points[self.cur_point_index]
        next_point = self.points[self.cur_point_index + 1]

        # divide by "frequency" to get the required velocity
        msg.linear.x = (next_point[0] - current_point[0])/self.frequency
        msg.linear.y = (next_point[1] - current_point[1])/self.frequency

        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'speed: {msg.linear.x}, {msg.linear.y}')
        self.cur_point_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = CircleTurtleNode()
    try:
        rclpy.spin(node)
    except:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
