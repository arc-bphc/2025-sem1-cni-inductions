import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
import math
import random
from functools import partial
import time

class Boid:
    def __init__(self, name, node, leader=False):
        self.name = name
        self.node = node
        self.pose = None
        self.velocity = [0.0, 0.0]
        self.leader = leader
        self.last_branch_time = time.time()
        self.branch_cooldown = random.uniform(5.0, 15.0)  # Time between branching attempts
        self.branch_chance = 0.2  # Probability of branching when conditions are met
        
        # Boid parameters (tunable)
        self.max_speed = 2.0
        self.min_speed = 0.5
        self.perception = 3.0
        self.separation_weight = 1.5
        self.alignment_weight = 1.0
        self.cohesion_weight = 1.0
        self.random_weight = 0.3  # Strength of random movement
        self.boundary_margin = 1.0
        
        # Create publisher for this boid
        self.publisher = self.node.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        
        # Create subscriber for this boid's pose
        self.node.create_subscription(Pose, f'/{self.name}/pose', self.pose_callback, 10)
        
        # Set random color for each boid
        self.set_random_color()
        
        # Initialize with random velocity
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(self.min_speed, self.max_speed)
        self.velocity = [math.cos(angle) * speed, math.sin(angle) * speed]

    def pose_callback(self, msg):
        self.pose = msg

    def set_random_color(self):
        # Create a client to set pen color
        client = self.node.create_client(SetPen, f'/{self.name}/set_pen')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('set_pen service not available, waiting again...')
        
        request = SetPen.Request()
        if self.leader:
            # Leaders are red
            request.r = 255
            request.g = 0
            request.b = 0
        else:
            # Followers get random colors
            request.r = random.randint(0, 255)
            request.g = random.randint(0, 255)
            request.b = random.randint(0, 255)
        request.width = 2
        request.off = 0
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.set_color_callback))

    def set_color_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {e}')

    def distance_to(self, other_boid):
        if self.pose is None or other_boid.pose is None:
            return float('inf')
            
        dx = self.pose.x - other_boid.pose.x
        dy = self.pose.y - other_boid.pose.y
        return math.sqrt(dx*dx + dy*dy)

    def separation(self, neighbors):
        steer = [0.0, 0.0]
        count = 0
        
        for neighbor in neighbors:
            dist = self.distance_to(neighbor)
            if dist > 0 and dist < self.perception:
                # Calculate repulsion force (inversely proportional to distance)
                dx = self.pose.x - neighbor.pose.x
                dy = self.pose.y - neighbor.pose.y
                
                # Normalize and weight by distance
                magnitude = math.sqrt(dx*dx + dy*dy)
                if magnitude > 0:
                    steer[0] += dx / magnitude / dist
                    steer[1] += dy / magnitude / dist
                    count += 1
        
        if count > 0:
            steer[0] /= count
            steer[1] /= count
            
        return steer

    def alignment(self, neighbors):
        if not neighbors:
            return [0.0, 0.0]
            
        avg_velocity = [0.0, 0.0]
        count = 0
        
        for neighbor in neighbors:
            dist = self.distance_to(neighbor)
            if dist < self.perception:
                avg_velocity[0] += neighbor.velocity[0]
                avg_velocity[1] += neighbor.velocity[1]
                count += 1
        
        if count > 0:
            avg_velocity[0] /= count
            avg_velocity[1] /= count
            
            # Return steering force toward average velocity
            return [
                avg_velocity[0] - self.velocity[0],
                avg_velocity[1] - self.velocity[1]
            ]
        
        return [0.0, 0.0]

    def cohesion(self, neighbors):
        if not neighbors:
            return [0.0, 0.0]
            
        center_of_mass = [0.0, 0.0]
        count = 0
        
        for neighbor in neighbors:
            dist = self.distance_to(neighbor)
            if dist < self.perception:
                center_of_mass[0] += neighbor.pose.x
                center_of_mass[1] += neighbor.pose.y
                count += 1
        
        if count > 0:
            center_of_mass[0] /= count
            center_of_mass[1] /= count
            
            # Return steering force toward center of mass
            return [
                center_of_mass[0] - self.pose.x,
                center_of_mass[1] - self.pose.y
            ]
        
        return [0.0, 0.0]

    def random_movement(self):
        # Add a small random vector to create more natural movement
        return [
            random.uniform(-1.0, 1.0) * self.random_weight,
            random.uniform(-1.0, 1.0) * self.random_weight
        ]

    def avoid_boundaries(self):
        steer = [0.0, 0.0]
        
        if self.pose.x < self.boundary_margin:
            steer[0] = 1.0
        elif self.pose.x > 11.0 - self.boundary_margin:
            steer[0] = -1.0
            
        if self.pose.y < self.boundary_margin:
            steer[1] = 1.0
        elif self.pose.y > 11.0 - self.boundary_margin:
            steer[1] = -1.0
            
        return steer

    def limit_velocity(self, velocity):
        speed = math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1])
        if speed > self.max_speed:
            velocity[0] = (velocity[0] / speed) * self.max_speed
            velocity[1] = (velocity[1] / speed) * self.max_speed
        elif speed < self.min_speed and speed > 0:
            velocity[0] = (velocity[0] / speed) * self.min_speed
            velocity[1] = (velocity[1] / speed) * self.min_speed
            
        return velocity

    def update(self, boids):
        if self.pose is None:
            return
            
        # Find neighbors (excluding self)
        neighbors = [boid for boid in boids if boid != self and boid.pose is not None]
        
        # Calculate steering forces from each rule
        separation_force = self.separation(neighbors)
        alignment_force = self.alignment(neighbors)
        cohesion_force = self.cohesion(neighbors)
        random_force = self.random_movement()
        boundary_force = self.avoid_boundaries()
        
        # Apply weights to each force
        separation_force = [x * self.separation_weight for x in separation_force]
        alignment_force = [x * self.alignment_weight for x in alignment_force]
        cohesion_force = [x * self.cohesion_weight for x in cohesion_force]
        random_force = [x * self.random_weight for x in random_force]
        boundary_force = [x * 2.0 for x in boundary_force]  # Strong boundary avoidance
        
        # Update velocity
        self.velocity[0] += separation_force[0] + alignment_force[0] + cohesion_force[0] + random_force[0] + boundary_force[0]
        self.velocity[1] += separation_force[1] + alignment_force[1] + cohesion_force[1] + random_force[1] + boundary_force[1]
        
        # Limit velocity
        self.velocity = self.limit_velocity(self.velocity)
        
        # Convert velocity to twist message
        twist_msg = Twist()
        
        # Calculate linear and angular velocities
        current_angle = math.atan2(self.velocity[1], self.velocity[0])
        angle_diff = current_angle - self.pose.theta
        
        # Normalize angle difference to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        speed = math.sqrt(self.velocity[0]*self.velocity[0] + self.velocity[1]*self.velocity[1])
        
        twist_msg.linear.x = speed
        twist_msg.angular.z = 4.0 * angle_diff  # Proportional control for turning
        
        # Publish the twist message
        self.publisher.publish(twist_msg)
        
        # Return whether this boid might branch (leaders only)
        if self.leader:
            return self.check_branching_conditions(neighbors)
        return False

    def check_branching_conditions(self, neighbors):
        current_time = time.time()
        
        # Check if cooldown has passed and random chance succeeds
        if (current_time - self.last_branch_time > self.branch_cooldown and 
            random.random() < self.branch_chance):
            
            # Check if we have enough neighbors to branch from
            if len(neighbors) >= 3:
                self.last_branch_time = current_time
                return True
                
        return False


class BoidsController(Node):
    def __init__(self):
        super().__init__('boids_controller')
        
        self.boids = []
        self.num_boids = 12  # Number of boids to spawn
        self.next_boid_id = 1
        self.max_boids = 20  # Maximum number of boids to prevent overcrowding
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')
            
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kill service not available, waiting again...')
        
        # Kill the default turtle
        self.kill_default_turtle()
        
        # Spawn initial boids
        self.spawn_initial_boids()
        
        # Create timer for updating boids
        self.timer = self.create_timer(0.1, self.timer_callback)

    def kill_default_turtle(self):
        request = Kill.Request()
        request.name = 'turtle1'
        future = self.kill_client.call_async(request)
        future.add_done_callback(partial(self.kill_callback, turtle_name=request.name))

    def kill_callback(self, future, turtle_name):
        try:
            response = future.result()
            self.get_logger().info(f'Killed turtle: {turtle_name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def spawn_initial_boids(self):
        # Spawn one leader
        self.spawn_boid(leader=True)
        
        # Spawn remaining boids as followers
        for i in range(self.num_boids - 1):
            self.spawn_boid(leader=False)

    def spawn_boid(self, leader=False):
        if len(self.boids) >= self.max_boids:
            return False
            
        # Spawn boid at random position
        request = Spawn.Request()
        request.x = random.uniform(2.0, 9.0)
        request.y = random.uniform(2.0, 9.0)
        request.theta = random.uniform(0, 2 * math.pi)
        request.name = f'boid{self.next_boid_id}'
        self.next_boid_id += 1
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(partial(self.spawn_callback, boid_name=request.name, leader=leader))
        return True

    def spawn_callback(self, future, boid_name, leader):
        try:
            response = future.result()
            leader_text = "leader" if leader else "follower"
            self.get_logger().info(f'Spawned {leader_text} boid: {boid_name}')
            
            # Create Boid object
            boid = Boid(boid_name, self, leader=leader)
            self.boids.append(boid)
            
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def timer_callback(self):
        # Update all boids and check for branching
        should_branch = False
        
        for boid in self.boids:
            branch = boid.update(self.boids)
            if branch:
                should_branch = True
                
        # If a leader wants to branch, spawn a new boid
        if should_branch and len(self.boids) < self.max_boids:
            # 50% chance the new boid will be a leader
            new_leader = random.random() < 0.5
            self.spawn_boid(leader=new_leader)
            self.get_logger().info(f"Branching: spawning new {'leader' if new_leader else 'follower'} boid")


def main(args=None):
    rclpy.init(args=args)
    node = BoidsController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()