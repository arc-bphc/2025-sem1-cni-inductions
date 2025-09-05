#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        
        self.declare_parameter('radius', 3.0)
        self.declare_parameter('num_points', 100)
        self.declare_parameter('timer_period', 0.1)
        
        self.radius = self.get_parameter('radius').value
        self.num_points = self.get_parameter('num_points').value
        self.timer_period = self.get_parameter('timer_period').value
        
        self.points = self.generate_circle_points()
        self.current_index = 0
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info('Circle drawer started with radius: {}'.format(self.radius))
        
    def generate_circle_points(self):
        points = []
        for i in range(self.num_points):
            theta = 2.0 * math.pi * i / self.num_points
            x = self.radius * math.cos(theta)
            y = self.radius * math.sin(theta)
            points.append((x, y))
        points.append(points[0])
        return points
        
    def control_loop(self):
        if self.current_index >= len(self.points) - 1:
            self.stop_turtle()
            self.get_logger().info('Circle drawing completed!')
            self.destroy_timer(self.timer)
            return
            
        # current and next points
        current_x, current_y = self.points[self.current_index]
        next_x, next_y = self.points[self.current_index + 1]
        
        # required velocities
        twist = Twist()
        twist.linear.x = (next_x - current_x) / self.timer_period
        twist.linear.y = (next_y - current_y) / self.timer_period
        twist.angular.z = 0.0 
        
        self.cmd_vel_publisher.publish(twist)
        self.current_index += 1
        
    def stop_turtle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
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
