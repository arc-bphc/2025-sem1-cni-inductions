#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose
import math

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        
        # Declare parameters
        self.declare_parameter('center_x', 5.5)
        self.declare_parameter('center_y', 8.5)
        self.declare_parameter('radius', 11.0)
        self.declare_parameter('num_points', 36)
        self.declare_parameter('linear_gain', 1.5)
        self.declare_parameter('angular_gain', 4.0)
        self.declare_parameter('max_linear_speed', 3.0)
        self.declare_parameter('max_angular_speed', 5.0)
        self.declare_parameter('target_threshold', 0.08)
        
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        self.num_points = self.get_parameter('num_points').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.target_threshold = self.get_parameter('target_threshold').value
        
        self.pose = None
        self.points = self.make_circle_points(self.center_x, self.center_y, self.radius, self.num_points)
        self.target_idx = 0
        
        # Create publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create subscriber
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Create service client
        self.clear_client = self.create_client(Empty, '/clear')
        
        # Wait for services
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear service not available, waiting...')
        
        # Clear screen and start
        self.clear_screen()
        
        self.get_logger().info('circle_drawer started')
        
    def make_circle_points(self, cx, cy, r, n):
        pts = []
        for i in range(n):
            theta = 2.0 * math.pi * float(i) / n
            pts.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
        pts.append(pts[0])  # close loop
        return pts
    
    def clear_screen(self):
        if self.clear_client.wait_for_service(timeout_sec=2.0):
            clear_request = Empty.Request()
            future = self.clear_client.call_async(clear_request)
            future.add_done_callback(self.clear_done_callback)
            
    def clear_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Cleared screen.')
            # Start the control loop after clearing
            self.timer = self.create_timer(0.05, self.control_loop)
        except Exception as e:
            self.get_logger().error(f'Failed to clear screen: {e}')

    def pose_callback(self, msg):
        self.pose = msg

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2*math.pi
        while d < -math.pi:
            d += 2*math.pi
        return d

    def control_loop(self):
        if self.pose is None:
            return
        if self.target_idx >= len(self.points):
            # stop
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Finished circle')
            self.destroy_timer(self.timer)
            return

        tx, ty = self.points[self.target_idx]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        

        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        ang_err = self.angle_diff(angle_to_target, self.pose.theta)

        twist = Twist()
        k_lin = self.linear_gain
        k_ang = self.angular_gain

        if dist > self.target_threshold:
            twist.linear.x = max(self.max_linear_speed, min(self.max_linear_speed, k_lin * dist))
            twist.angular.z = max(self.max_angular_speed, min(self.max_angular_speed, k_ang * ang_err))
        else:
            # reached point -> go to next
            self.target_idx += 1
            twist = Twist()  # small pause
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
