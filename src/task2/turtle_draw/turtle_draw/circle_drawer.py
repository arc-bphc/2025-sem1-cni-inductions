import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class CircleTurtleNode(Node):
	def __init__(self):
		super().__init__('circle_turtle_node')
		self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
		self.subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
		self.timer = self.create_timer(1.0, self.publish_cmd)
		self.pose = None
		self.current_index = 0
		self.points = []
		#Circle (comment out if square)
		self.generate_points_circle(1, 50)
		#Square (comment out if circle)
		#self.generate_points_square(2)

	def pose_callback(self, msg):
		self.pose = msg

	def generate_points_circle(self, radius, divisions):
		for i in range(divisions):
			x = radius * math.cos(2*math.pi*i/divisions) + 5.5
			y = radius * math.sin(2*math.pi*i/divisions) + 5.5
			self.points.append((x,y))

		self.points.append(self.points[0])

	def generate_points_square(self, side):
		x = side/2 + 5.5
		y = side/2 + 5.5
		x_neg = 5.5 - side/2
		y_neg = 5.5 - side/2
		self.points.append((x,y))
		self.points.append((x_neg,y))
		self.points.append((x_neg,y_neg))
		self.points.append((x,y_neg))
		self.points.append((x,y))

	def publish_cmd(self):	
		if self.pose is None or self.current_index >= len(self.points): return None

		x, y = self.points[self.current_index]
		dx = x - self.pose.x
		dy = y - self.pose.y
		distance = math.sqrt(dx**2 + dy**2)
		angle = math.atan2(dy, dx)
		d_angle = self.normalize_angle(angle - self.pose.theta)

		msg = Twist()
		if abs(d_angle) > 0.1:
			msg.linear.x = 0.0
			msg.angular.z = min(0.8 * d_angle, 0.8)
		else:
			msg.angular.z = 0.0
			msg.linear.x = min(distance, 2.0)

		self.publisher.publish(msg)

		if distance < 0.1:
			self.current_index += 1
			self.get_logger().info(f'Moved to point {self.current_index + 1}')

	def normalize_angle(self, angle):
		while angle > math.pi:
			angle -= 2 * math.pi
		while angle < -math.pi:
			angle += 2 * math.pi
		return angle

def main(args=None):
	rclpy.init(args=args)
	node = CircleTurtleNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
