#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import math, random
from functools import partial

def clamp(v, lo, hi):
	return max(lo, min(hi, v))

def norm_angle(a):
	while a > math.pi:
		a -= 2 * math.pi
	while a < -math.pi:
		a += 2 * math.pi
	return a

class SwarmNode(Node):
	def __init__(self):
		super().__init__('turtle_swarm')

		# ---- Tuned Parameters ----
		self.declare_parameter('target_count', 5)
		self.declare_parameter('neighbor_radius', 1.6)
		self.declare_parameter('max_speed', 1.5)
		self.declare_parameter('dt', 0.1)

		# Behavior weights
		self.declare_parameter('w_sep', 3.0)     # stronger separation
		self.declare_parameter('w_align', 1.2)   # slightly stronger alignment
		self.declare_parameter('w_cohes', 0.8)   # weaker cohesion to prevent clumping
		self.declare_parameter('avoid_wall', 2.0)
		self.declare_parameter('wall_margin', 1.0)
		self.declare_parameter('k_ang', 6.0)

		# Load params
		self.target_count = int(self.get_parameter('target_count').value)
		self.neighbor_radius = float(self.get_parameter('neighbor_radius').value)
		self.max_speed = float(self.get_parameter('max_speed').value)
		self.dt = float(self.get_parameter('dt').value)

		self.w_sep = float(self.get_parameter('w_sep').value)
		self.w_align = float(self.get_parameter('w_align').value)
		self.w_cohes = float(self.get_parameter('w_cohes').value)
		self.avoid_wall = float(self.get_parameter('avoid_wall').value)
		self.wall_margin = float(self.get_parameter('wall_margin').value)
		self.k_ang = float(self.get_parameter('k_ang').value)

		# name -> {'pose':Pose, 'vel':[vx,vy], 'pub':Publisher}
		self.agents = {}

		# wait for /spawn
		self.spawn_client = self.create_client(Spawn, 'spawn')
		if not self.spawn_client.wait_for_service(timeout_sec=5.0):
			self.get_logger().error('spawn service not available. Is turtlesim_node running?')
			raise RuntimeError('spawn not available')

		# ensure turtles exist
		self.ensure_turtles()

		# control loop
		self.timer = self.create_timer(self.dt, self.update)
		self.get_logger().info('Swarm started.')

	def ensure_turtles(self):
		names = ['turtle1']
		i = 2
		while len(names) < self.target_count:
			candidate = f'turtle{i}'
			req = Spawn.Request()
			req.x = float(random.uniform(1.0, 10.5))
			req.y = float(random.uniform(1.0, 10.5))
			req.theta = float(random.uniform(-math.pi, math.pi))
			req.name = candidate
			fut = self.spawn_client.call_async(req)
			rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
			if fut.result() is not None:
				names.append(fut.result().name)
				self.get_logger().info(f"Spawned {fut.result().name}")
			i += 1

		for n in names:
			if n not in self.agents:
				pub = self.create_publisher(Twist, f'/{n}/cmd_vel', 10)
				sub = self.create_subscription(Pose, f'/{n}/pose', partial(self.pose_cb, n), 10)
				self.agents[n] = {
					'pose': None,
					'vel': [random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)],
					'pub': pub
				}

	def pose_cb(self, name, msg: Pose):
		if name in self.agents:
			self.agents[name]['pose'] = msg

	def update(self):
		# wait until all pose data available
		if not all(self.agents[n]['pose'] is not None for n in self.agents):
			return

		names = list(self.agents.keys())
		pos = {n: (self.agents[n]['pose'].x, self.agents[n]['pose'].y) for n in names}
		vel = {n: (self.agents[n]['vel'][0], self.agents[n]['vel'][1]) for n in names}

		for n in names:
			x, y = pos[n]
			vx, vy = list(vel[n])
			sep = [0.0, 0.0]
			align = [0.0, 0.0]
			coh = [0.0, 0.0]
			cnt = 0

			for m in names:
				if m == n:
					continue
				mx, my = pos[m]
				dx, dy = mx - x, my - y
				d = math.hypot(dx, dy)
				if d < 1e-6 or d > self.neighbor_radius:
					continue

				# separation (push away, stronger now)
				sep[0] -= dx / d
				sep[1] -= dy / d
				# alignment
				align[0] += vel[m][0]
				align[1] += vel[m][1]
				# cohesion
				coh[0] += mx
				coh[1] += my
				cnt += 1

			if cnt > 0:
				align[0] = (align[0] / cnt) - vx
				align[1] = (align[1] / cnt) - vy
				coh[0] = (coh[0] / cnt) - x
				coh[1] = (coh[1] / cnt) - y
				# separation is no longer averaged → stronger effect

			# walls (turtlesim ~0..11)
			wx = wy = 0.0
			if x < self.wall_margin:
				wx += (self.wall_margin - x)
			if x > 11 - self.wall_margin:
				wx -= (x - (11 - self.wall_margin))
			if y < self.wall_margin:
				wy += (self.wall_margin - y)
			if y > 11 - self.wall_margin:
				wy -= (y - (11 - self.wall_margin))

			ax = (self.w_sep * sep[0] + self.w_align * align[0] +
				  self.w_cohes * coh[0] + self.avoid_wall * wx + random.uniform(-0.05, 0.05))
			ay = (self.w_sep * sep[1] + self.w_align * align[1] +
				  self.w_cohes * coh[1] + self.avoid_wall * wy + random.uniform(-0.05, 0.05))

			vx += ax * self.dt
			vy += ay * self.dt

			speed = math.hypot(vx, vy)
			if speed > self.max_speed:
				s = self.max_speed / speed
				vx *= s
				vy *= s

			# store vel
			self.agents[n]['vel'][0] = vx
			self.agents[n]['vel'][1] = vy

			desired = math.atan2(vy, vx)
			cur = self.agents[n]['pose'].theta
			ang_err = norm_angle(desired - cur)

			twist = Twist()
			# smoother forward scaling
			forward = clamp(speed * max(0.4, 1.0 - abs(ang_err) / 2.0), 0.0, self.max_speed)
			twist.linear.x = forward
			twist.angular.z = clamp(self.k_ang * ang_err, -6.0, 6.0)
			self.agents[n]['pub'].publish(twist)

def main(args=None):
	rclpy.init(args=args)
	node = SwarmNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()
