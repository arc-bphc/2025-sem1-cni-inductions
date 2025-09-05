import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareTurtleNode(Node):
    def __init__(self):
        super().__init__('square_turtle_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Control loop timer (10 Hz for smoother control)
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # State machine variables
        self.state = "FORWARD"   # start by moving forward
        self.steps = 0
        self.side_count = 0

        # Tunable parameters
        self.forward_steps = 50   # steps to move one side
        self.turn_steps = 16     # steps to turn 90 degrees

    def publish_cmd(self):
        msg = Twist()

        if self.state == "FORWARD":
            msg.linear.x = 1.0
            msg.angular.z = 0.0
            self.steps += 1
            if self.steps >= self.forward_steps:
                # Finished one side, start turning
                self.steps = 0
                self.state = "TURN"
                self.get_logger().info(f"Finished side {self.side_count + 1}, turning...")

        elif self.state == "TURN":
            msg.linear.x = 0.0
            msg.angular.z = 1.0  # rotate
            self.steps += 1
            if self.steps >= self.turn_steps:
                # Finished turning 90 degrees
                self.steps = 0
                self.side_count += 1
                if self.side_count >= 4:
                    self.get_logger().info("Finished square!")
                    rclpy.shutdown()
                else:
                    self.state = "FORWARD"
                    self.get_logger().info("Moving forward again...")

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquareTurtleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
