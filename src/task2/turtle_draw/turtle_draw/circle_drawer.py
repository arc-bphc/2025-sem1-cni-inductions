import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from math import pi, sin, cos

class CircleTurtleNode(Node):
    def __init__(self):
        super().__init__('circle_turtle_node')
        self.circle_time_period = 5
        self.circle_radius = 1
        self.angular_speed = 2*pi/self.circle_time_period
        self.linear_speed = self.angular_speed*self.circle_radius
        self.start = time.time()
        
        # publisher
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # timer
        self.timer = self.create_timer(1.0, self.publish_cmd)

    def publish_cmd(self):
        t = time.time() - self.start
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'time: {t}, speed: {msg.linear.x}, {msg.linear.y}')

def main(args=None):
    rclpy.init(args=args)
    node = CircleTurtleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
