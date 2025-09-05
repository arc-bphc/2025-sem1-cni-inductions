import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import math

class SpiralDrawer(Node):
    def __init__(self):
        super().__init__('spiral_drawer')
        
        # Declare parameters
        self.declare_parameter('center_x', 5.5)
        self.declare_parameter('center_y', 5.5)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('angular_speed', 2.0)
        self.declare_parameter('radius_growth', 0.01)
        self.declare_parameter('base_linear_speed', 1.0)
        self.declare_parameter('max_radius', 3.0)
        
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.initial_theta = self.get_parameter('initial_theta').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.radius_growth = self.get_parameter('radius_growth').value
        self.base_linear_speed = self.get_parameter('base_linear_speed').value
        self.max_radius = self.get_parameter('max_radius').value
        
        # Internal state
        self.theta = self.initial_theta
        self.radius = 0.0
        self.spiral_completed = False
        
        # Create publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create service clients
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.clear_client = self.create_client(Empty, '/clear')
        
        # Wait for services
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting...')
            
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear service not available, waiting...')
        
        # Initialize spiral
        self.teleport_to_center()
        
    def teleport_to_center(self):
        """Teleport turtle to the center of the screen"""
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = self.center_x
        teleport_request.y = self.center_y
        teleport_request.theta = self.initial_theta
        
        future = self.teleport_client.call_async(teleport_request)
        future.add_done_callback(self.teleport_done_callback)
        
    def teleport_done_callback(self, future):
        """Callback when teleport service completes"""
        try:
            response = future.result()
            self.get_logger().info('Teleported turtle to center for spiral')
            self.clear_screen()
        except Exception as e:
            self.get_logger().error(f'Failed to teleport turtle: {e}')
            
    def clear_screen(self):
        """Clear the turtlesim screen"""
        clear_request = Empty.Request()
        future = self.clear_client.call_async(clear_request)
        future.add_done_callback(self.clear_done_callback)
            
    def clear_done_callback(self, future):
        """Callback when clear service completes"""
        try:
            response = future.result()
            self.get_logger().info('Screen cleared, starting spiral drawing')
            # Start the control loop after clearing
            self.timer = self.create_timer(0.1, self.control_loop)
        except Exception as e:
            self.get_logger().error(f'Failed to clear screen: {e}')
            
    def control_loop(self):
        """Main control loop to draw a spiral"""
        if self.spiral_completed:
            return
            
        # Check if spiral is completed (reached max radius)
        if self.radius >= self.max_radius:
            self.spiral_completed = True
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Spiral completed! Stopping turtle.')
            self.destroy_timer(self.timer)
            return
        
        # Create control command for spiral motion
        twist = Twist()
        
        # Linear speed increases with radius for spiral effect
        twist.linear.x = float(self.base_linear_speed + self.radius * 0.9)
        
        # Constant angular speed
        twist.angular.z = float(self.angular_speed)
        
        # Publish control command
        self.cmd_vel_publisher.publish(twist)
        
        # Update spiral parameters
        self.radius += self.radius_growth
        self.theta += self.angular_speed * 0.1  # 0.1 is the timer period

def main(args=None):
    rclpy.init(args=args)
    node = SpiralDrawer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
