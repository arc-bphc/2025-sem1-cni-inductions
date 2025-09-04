import rclpy      # main python library for ros2
from rclpy.node import Node      # imports Node class
from geometry_msgs.msg import Twist  # used for linear and angular velocity etc , defines twist

class CircleDrawer(Node):
    def __init__(self):     # dunder method automatically calls when creating an instance

        super().__init__('circle_drawer') # it call the Node constructor with node name
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # we need a publisher when we use ros as its used to send out info
        self.radius = 1.0           # fix radius of circle
        self.linear_speed = 1.0         # constant speed
        self.angular_speed = self.linear_speed / self.radius  # angular speed to keep circular path
        self.twist = Twist()            # twist message for velocity
        self.twist.linear.x = self.linear_speed    # we are defining these two here as they arent needed to be called everytime
        self.twist.angular.z = self.angular_speed
        self.timer = self.create_timer(0.1, self.timer_callback)  # timer calls method every 0.1 second

    def timer_callback(self):     
        
        self.publisher_.publish(self.twist)  # publish constant velocity command to move circle
        self.get_logger().info(f'Linear speed: {self.linear_speed:.2f}, Angular speed: {self.angular_speed:.2f}')


def main(args=None):        # accepts
    rclpy.init(args=args)      # initializes ros2 python library
    node = CircleDrawer()      # creates instance

    try:
        rclpy.spin(node)        # keeps the node running
    except KeyboardInterrupt:      # built-in automatically understands ctrl+C exit
        pass
    finally:
        # stop turtle
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher_.publish(node.twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # entry point
    main()
