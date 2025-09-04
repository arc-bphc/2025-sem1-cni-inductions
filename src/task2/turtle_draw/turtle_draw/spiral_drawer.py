import rclpy       # main python library for ros2
from rclpy.node import Node  # imports node class
from geometry_msgs.msg import Twist   # used for linear and angular velocity etc , defines twist

class SpiralDrawer(Node):
    def __init__(self):    # dunder method automatically calls when creating an instance
 
        super().__init__('spiral_drawer')  # it calls the Node constructor with node name
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # we need a publisher when we use ros as its used to send out info
        self.linear_speed = 0.0
        self.angular_speed = 2.5
        self.linear_acceleration = 0.05   # spiral is just a circle with increasing radius so we introduce acceleration 
        self.twist = Twist()     # the twist message in ros2 is used to specify velocity of a turtle in free space
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        
        self.linear_speed += self.linear_acceleration
        self.twist.linear.x = self.linear_speed    # we are defining these two here as they are needed to be called everytime
        self.twist.angular.z = self.angular_speed
        self.publisher_.publish(self.twist)     # publish command to move spiral
        self.get_logger().info(f'Linear speed: {self.linear_speed:.2f}, Angular speed: {self.angular_speed}')

def main(args=None):    # accepts
    rclpy.init(args=args)   # initializes ros2 python library
    node = SpiralDrawer()   # creates instance

    try:
        rclpy.spin(node)      # keeps the node running
    except KeyboardInterrupt:    # built-in automatically understands ctrl+C exit
        pass
    finally:
        # Stop turtle
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher_.publish(node.twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':    # entry point
