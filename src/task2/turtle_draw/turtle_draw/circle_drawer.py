import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  
import math
import time
class PublishingaCircle(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Publisher node started!")

    def draw_circle(self):
        msg = Twist()
        theta = 0
        step = 0.1
        r = 2.0
        while rclpy.ok():
            x = r * math.cos(theta)
            y = r * math.sin(theta)

            theta_next = theta + step
            x_next = r * math.cos(theta_next)
            y_next = r * math.sin(theta_next)

            dx = x_next - x
            dy = y_next - y

            dist = math.sqrt(dx**2 + dy**2)
            v = dist/0.1

            msg.linear.x = v
            msg.angular.z = v/r
            self.publisher_.publish(msg)
            self.get_logger().info(f"Moving to ({x_next:.2f}, {y_next:.2f})")

            theta = theta_next
            time.sleep(0.1)



def main(args=None):
    rclpy.init(args=args)
    node = PublishingaCircle()
    try:
        node.draw_circle()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()