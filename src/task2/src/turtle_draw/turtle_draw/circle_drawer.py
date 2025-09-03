import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Publish velocity commands every 0.1s
        self.timer = self.create_timer(0.1, self.move)

    def move(self):
        msg = Twist()
        msg.linear.x = 2.0    # forward speed
        msg.angular.z = 1.0   # turning speed
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)   # keep running
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
