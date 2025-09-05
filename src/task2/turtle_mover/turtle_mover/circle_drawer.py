import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.publish_circle_velocity)
        self.angular_speed = 0.5
        self.radius = 2.0
        self.theta = 0.0

    def publish_circle_velocity(self):
        twist = Twist()
        twist.linear.x = self.radius * self.angular_speed
        twist.angular.z = self.angular_speed
        self.vel_pub.publish(twist)
        self.theta += self.angular_speed * self.timer_period

def main(args=None):
    rclpy.init(args=args)
    mover = CircleMover()
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
