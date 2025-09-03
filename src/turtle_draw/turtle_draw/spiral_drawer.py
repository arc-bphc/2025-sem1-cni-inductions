import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpiralDrawer(Node):
    def __init__(self):
        super().__init__('spiral_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.linear_speed = 0.0   
        self.angular_speed = 2.5
        self.linear_acceleration = 0.05
        self.twist = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        
        self.linear_speed += self.linear_acceleration
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed
        self.publisher_.publish(self.twist)
        self.get_logger().info(f'Linear speed: {self.linear_speed:.2f}, Angular speed: {self.angular_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = SpiralDrawer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the turtle
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher_.publish(node.twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
