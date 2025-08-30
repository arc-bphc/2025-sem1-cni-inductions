import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDrawerNode(Node):
    def __init__(self):
        # Change the node name for clarity
        super().__init__('circle_drawer_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Publish more frequently for a smoother circle
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Circle Drawer Node has been started!')

    def timer_callback(self):
        msg = Twist()

        # Set a constant forward speed
        msg.linear.x = 2.0

        # Set a constant turning speed to create a circle
        msg.angular.z = 1.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()