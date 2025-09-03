import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import random

class BoidSwarm(Node):
	def __init__(self):
		super().__init__('boid_swarm')
		self.num_boids = 4
		self.boid_names = [f'turtle{i+2}' for i in range(self.num_boids)]
		self.poses = {}
		self.boid_publishers = {}

		self.spawn_boids()
		self.create_timer(0.1, self.update_swarm)

	def spawn_boids(self):
		client = self.create_client(Spawn, '/spawn')
		while not client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for spawn service')

		for name in self.boid_names:
			req = Spawn.Request()
			req.x = random.uniform(2.0, 9.0)
			req.y = random.uniform(2.0, 9.0)
			req.theta = random.uniform(0, 2 * math.pi)
			req.name = name
			future = client.call_async(req)
			self.boid_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
			self.create_subscription(Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), 10)

		self.boid_names = ['turtle1'] + self.boid_names
		self.boid_publishers['turtle1'] = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.create_subscription(Pose, '/turtle1/pose', lambda msg, n='turtle1': self.pose_callback(msg, n), 10)

	def pose_callback(self, msg, name):
		self.poses[name] = msg

	def update_swarm(self):
		for name in self.boid_names:
			if name not in self.poses:
				continue

			pose = self.poses[name]
			neighbours = [p for n, p in self.poses.items() if n != name]

			if not neighbours:
				continue

			#Boid rules

			sep_x, sep_y = self.separation(pose, neighbours)
			coh_x, coh_y = self.cohesion(pose, neighbours)
			align_theta = self.alignment(pose, neighbours)

			vel_x = 1.25 * sep_x + 0.75 * coh_x
			vel_y = 1.25 * sep_y + 0.75 *  coh_y
			bx, by = self.boundary_force(pose)
			vel_x += bx
			vel_y += by
			target_theta = math.atan2(vel_y, vel_x)
			angle_diff = self.normalize_angle(target_theta - pose.theta)

			msg = Twist()
			msg.linear.x = 2.0
			msg.angular.z = min(2.0 * angle_diff, 2.0)
			self.boid_publishers[name].publish(msg)

	def boundary_force(self, pose):
		force_x, force_y = 0.0, 0.0
		margin = 1.5
		strength = 1.0

		if pose.x < margin:
			force_x += strength
		elif pose.x > 11 - margin:
			force_x -= strength

		if pose.y < margin:
			force_y += strength
		elif pose.y > 11 - margin:
			force_y -= strength

		return force_x, force_y

	def separation(self, pose, neighbours):
		dx, dy = 0.0, 0.0
		for other in neighbours:
			dist = math.sqrt((pose.x - other.x)**2 + (pose.y - other.y)**2)
			if dist < 1.0:
				dx += pose.x - other.x
				dy += pose.y - other.y

		return dx, dy

	def cohesion(self, pose, neighbours):
		avg_x = sum(p.x for p in neighbours) / len(neighbours)
		avg_y = sum(p.y for p in neighbours) / len(neighbours)
		return avg_x - pose.x, avg_y - pose.y

	def alignment(self, pose, neighbours):
		avg_theta = sum(p.theta for p in  neighbours) / len(neighbours)
		return avg_theta

	def normalize_angle(self, angle):
		while angle > math.pi:
			angle -= 2 * math.pi

		while angle < -math.pi:
			angle += 2 * math.pi

		return angle

def main(args=None):
	rclpy.init(args=args)
	node = BoidSwarm()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
