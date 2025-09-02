import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinimalTurtleNode(Node):
    def __init__(self):
        super().__init__('minimal_turtle_node')
        # Publisher to control the turtle
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Timer to call publish_cmd every second
        self.timer = self.create_timer(1.0, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 1.0   # move forward
        msg.angular.z = 0.0  # no rotation
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: forward command')


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
