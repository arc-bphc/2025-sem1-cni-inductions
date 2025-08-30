import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class StarDrawerNode(Node):
    def __init__(self):
        super().__init__('star_drawer_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # States: 'forward', 'turn', 'stop'
        self.state = 'forward'
        self.state_start_time = time.time()
        self.points_drawn = 0
        
        # Parameters for the star
        self.linear_speed = 2.5
        self.angular_speed = 1.5  # rad/s
        self.side_duration = 1.5  # seconds
        
        # A 5-pointed star requires a 144-degree turn (2.513 radians)
        turn_angle_rad = 144 * (math.pi / 180)
        self.turn_duration = turn_angle_rad / self.angular_speed
        
        self.get_logger().info('Star Drawer node started. Drawing a 5-pointed star.')

    def timer_callback(self):
        msg = Twist()
        elapsed_time = time.time() - self.state_start_time

        if self.state == 'forward':
            if elapsed_time < self.side_duration:
                msg.linear.x = self.linear_speed
            else:
                self.state = 'turn'
                self.state_start_time = time.time()
        
        elif self.state == 'turn':
            if elapsed_time < self.turn_duration:
                msg.angular.z = self.angular_speed
            else:
                self.points_drawn += 1
                if self.points_drawn >= 5:
                    self.state = 'stop'
                    self.get_logger().info('Star complete!')
                    self.timer.cancel()  # Stop the timer
                else:
                    self.state = 'forward'
                    self.state_start_time = time.time()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StarDrawerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()