#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random
import numpy as np

class Boid:
    def __init__(self, name, node):
        self.name = name
        self.node = node
        self.pose = Pose()
        self.velocity = Twist()
        self.max_speed = 2.0
        self.max_force = 0.5
        
        # Create publisher for this turtle
        self.publisher = node.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        
        # Subscribe to pose topic
        node.create_subscription(Pose, f'/{self.name}/pose', self.pose_callback, 10)
    
    def pose_callback(self, msg):
        self.pose = msg
        
    def apply_force(self, force):
        # Limit the force
        force_magnitude = math.sqrt(force.linear.x**2 + force.linear.y**2)
        if force_magnitude > self.max_force:
            force.linear.x = (force.linear.x / force_magnitude) * self.max_force
            force.linear.y = (force.linear.y / force_magnitude) * self.max_force
        
        # Apply the force to velocity
        self.velocity.linear.x += force.linear.x
        self.velocity.linear.y += force.linear.y
        
        # Limit the velocity
        velocity_magnitude = math.sqrt(self.velocity.linear.x**2 + self.velocity.linear.y**2)
        if velocity_magnitude > self.max_speed:
            self.velocity.linear.x = (self.velocity.linear.x / velocity_magnitude) * self.max_speed
            self.velocity.linear.y = (self.velocity.linear.y / velocity_magnitude) * self.max_speed
            
        # Add some angular velocity to make it look more natural
        self.velocity.angular.z = math.atan2(self.velocity.linear.y, self.velocity.linear.x)
    
    def update(self):
        self.publisher.publish(self.velocity)

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        self.boids = []
        self.num_turtles = 5
        self.perception_radius = 3.0
        
        # Services
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.set_pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        
        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')
        
        # Kill the default turtle
        self.kill_default_turtle()
        
        # Spawn turtles
        self.spawn_turtles()
        
        # Timer for updating boids
        self.timer = self.create_timer(0.1, self.update_boids)
    
    def kill_default_turtle(self):
        req = Kill.Request()
        req.name = 'turtle1'
        future = self.kill_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
    
    def spawn_turtles(self):
        for i in range(self.num_turtles):
            req = Spawn.Request()
            req.x = random.uniform(1.0, 10.0)
            req.y = random.uniform(1.0, 10.0)
            req.theta = random.uniform(0.0, 2 * math.pi)
            req.name = f'turtle{i+1}'
            
            future = self.spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info(f"Spawned {req.name}")
                # Create boid object
                boid = Boid(req.name, self)
                # Set random initial velocity
                boid.velocity.linear.x = random.uniform(-1.0, 1.0)
                boid.velocity.linear.y = random.uniform(-1.0, 1.0)
                self.boids.append(boid)
            else:
                self.get_logger().error(f"Failed to spawn {req.name}")
    
    def get_neighbors(self, boid):
        neighbors = []
        for other in self.boids:
            if other.name != boid.name:
                distance = math.sqrt((boid.pose.x - other.pose.x)**2 + 
                                    (boid.pose.y - other.pose.y)**2)
                if distance < self.perception_radius:
                    neighbors.append(other)
        return neighbors
    
    def separation(self, boid, neighbors):
        steering = Twist()
        total = 0
        
        for neighbor in neighbors:
            distance = math.sqrt((boid.pose.x - neighbor.pose.x)**2 + 
                                (boid.pose.y - neighbor.pose.y)**2)
            if distance > 0:
                diff = Twist()
                diff.linear.x = boid.pose.x - neighbor.pose.x
                diff.linear.y = boid.pose.y - neighbor.pose.y
                # Weight by distance
                diff.linear.x /= distance
                diff.linear.y /= distance
                
                steering.linear.x += diff.linear.x
                steering.linear.y += diff.linear.y
                total += 1
        
        if total > 0:
            steering.linear.x /= total
            steering.linear.y /= total
            # Normalize
            magnitude = math.sqrt(steering.linear.x**2 + steering.linear.y**2)
            if magnitude > 0:
                steering.linear.x /= magnitude
                steering.linear.y /= magnitude
                steering.linear.x *= boid.max_speed
                steering.linear.y *= boid.max_speed
                # Subtract current velocity
                steering.linear.x -= boid.velocity.linear.x
                steering.linear.y -= boid.velocity.linear.y
        
        return steering
    
    def alignment(self, boid, neighbors):
        steering = Twist()
        total = 0
        
        for neighbor in neighbors:
            steering.linear.x += neighbor.velocity.linear.x
            steering.linear.y += neighbor.velocity.linear.y
            total += 1
        
        if total > 0:
            steering.linear.x /= total
            steering.linear.y /= total
            # Normalize
            magnitude = math.sqrt(steering.linear.x**2 + steering.linear.y**2)
            if magnitude > 0:
                steering.linear.x /= magnitude
                steering.linear.y /= magnitude
                steering.linear.x *= boid.max_speed
                steering.linear.y *= boid.max_speed
                # Subtract current velocity
                steering.linear.x -= boid.velocity.linear.x
                steering.linear.y -= boid.velocity.linear.y
        
        return steering
    
    def cohesion(self, boid, neighbors):
        steering = Twist()
        total = 0
        
        for neighbor in neighbors:
            steering.linear.x += neighbor.pose.x
            steering.linear.y += neighbor.pose.y
            total += 1
        
        if total > 0:
            steering.linear.x /= total
            steering.linear.y /= total
            # Seek towards the average position
            desired = Twist()
            desired.linear.x = steering.linear.x - boid.pose.x
            desired.linear.y = steering.linear.y - boid.pose.y
            # Normalize
            magnitude = math.sqrt(desired.linear.x**2 + desired.linear.y**2)
            if magnitude > 0:
                desired.linear.x /= magnitude
                desired.linear.y /= magnitude
                desired.linear.x *= boid.max_speed
                desired.linear.y *= boid.max_speed
                # Subtract current velocity
                steering.linear.x = desired.linear.x - boid.velocity.linear.x
                steering.linear.y = desired.linear.y - boid.velocity.linear.y
        
        return steering
    
    def update_boids(self):
        for boid in self.boids:
            neighbors = self.get_neighbors(boid)
            
            if neighbors:
                # Apply Boids rules
                separation = self.separation(boid, neighbors)
                alignment = self.alignment(boid, neighbors)
                cohesion = self.cohesion(boid, neighbors)
                
                # Weight the forces
                separation.linear.x *= 1.5
                separation.linear.y *= 1.5
                alignment.linear.x *= 1.0
                alignment.linear.y *= 1.0
                cohesion.linear.x *= 1.0
                cohesion.linear.y *= 1.0
                
                # Apply forces
                boid.apply_force(separation)
                boid.apply_force(alignment)
                boid.apply_force(cohesion)
            
            # Boundary conditions - wrap around
            if boid.pose.x < 0.0:
                boid.pose.x = 11.0
            elif boid.pose.x > 11.0:
                boid.pose.x = 0.0
                
            if boid.pose.y < 0.0:
                boid.pose.y = 11.0
            elif boid.pose.y > 11.0:
                boid.pose.y = 0.0
            
            # Update the boid
            boid.update()

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmController()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()