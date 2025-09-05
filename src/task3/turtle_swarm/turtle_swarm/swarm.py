import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random
import time
from typing import List, Dict

class Boid:
    def __init__(self, name, x, y, theta):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.pen_set = False  # flag if pen color has been set or not
        
    def update_pose(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta
        self.velocity = pose.linear_velocity
        self.angular_velocity = pose.angular_velocity

class BoidsSwarmNode(Node):
    def __init__(self):
        super().__init__('swarm')
        
        # declaring parameters
        self.declare_parameter('num_turtles', 5)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('neighbor_distance', 3.0)
        self.declare_parameter('separation_weight', 1.5)
        self.declare_parameter('alignment_weight', 1.0)
        self.declare_parameter('cohesion_weight', 1.0)
        self.declare_parameter('boundary_weight', 2.0)
        self.declare_parameter('boundary_margin', 1.0)
        
        self.num_turtles = self.get_parameter('num_turtles').value
        self.max_speed = self.get_parameter('max_speed').value
        self.neighbor_distance = self.get_parameter('neighbor_distance').value
        self.separation_weight = self.get_parameter('separation_weight').value
        self.alignment_weight = self.get_parameter('alignment_weight').value
        self.cohesion_weight = self.get_parameter('cohesion_weight').value
        self.boundary_weight = self.get_parameter('boundary_weight').value
        self.boundary_margin = self.get_parameter('boundary_margin').value
        
        self.boids: Dict[str, Boid] = {} #tracking the turtetl
        self.pose_subscribers = []
        self.cmd_vel_publishers = {}
        
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
            
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting...')
            
        self.init_swarm()
        
    def init_swarm(self):
        # kill the default turtle if it exists
        self.kill_default_turtle()       

    def kill_default_turtle(self):
        # (more ideally you would kill all turtles that exist and reset and then create it anew but it bugs out and this basically works so.)

        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        future = self.kill_client.call_async(kill_request)
        future.add_done_callback(self.turtle_killed_callback)
        
    def turtle_killed_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Default turtle killed, spawning new turtles...')
            for i in range(self.num_turtles):
                # small delay between spawns to avoid conflicts
                time.sleep(0.2)
                self.spawn_turtle(f'turtle{i+1}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to kill default turtle: {e}')
            # (try to) spawn turtles anyway (maybe turtle1 doesn't exist)
            for i in range(self.num_turtles):
                time.sleep(0.2)
                self.spawn_turtle(f'turtle{i+1}')
                
    def spawn_turtle(self, name):
        spawn_request = Spawn.Request()
        spawn_request.name = name
        spawn_request.x = random.uniform(2.0, 9.0)
        spawn_request.y = random.uniform(2.0, 9.0)
        spawn_request.theta = random.uniform(0.0, 2 * math.pi)
        
        future = self.spawn_client.call_async(spawn_request)
        future.add_done_callback(lambda future, n=name: self.turtle_spawned_callback(future, n))
        
    def turtle_spawned_callback(self, future, name):
        try:
            response = future.result()
            if response.name != '':
                self.get_logger().info(f'Spawned turtle: {name}')
                
                self.boids[name] = Boid(name, 0, 0, 0)
                
                # subscriber for pose
                sub = self.create_subscription(
                    Pose,
                    f'/{name}/pose',
                    lambda msg, n=name: self.pose_callback(msg, n),
                    10)
                self.pose_subscribers.append(sub)
                
                pub = self.create_publisher(
                    Twist,
                    f'/{name}/cmd_vel',
                    10)
                self.cmd_vel_publishers[name] = pub
                
                if len(self.boids) == 1 and not hasattr(self, 'timer'):
                    self.timer = self.create_timer(0.1, self.update_boids)
                
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle {name}: {e}')
            # try to spawn with a different name if there's a conflict
            if "already exists" in str(e):
                new_name = f"{name}_{random.randint(1000, 9999)}"
                self.get_logger().info(f'Retrying with name: {new_name}')
                self.spawn_turtle(new_name)
            
    def set_pen_color(self, name):
        colors = {
            'turtle1': (255, 0, 0),     # Red
            'turtle2': (0, 255, 0),     # Green
            'turtle3': (0, 0, 255),     # Blue
            'turtle4': (255, 255, 0),   # Yellow
            'turtle5': (255, 0, 255),   # Magenta
        }
        
        # handle alternative names if we rename due to conflicts
        base_name = name.split('_')[0]  # base name without random suffix
        if base_name in colors:
            color = colors[base_name]
        else:
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        
        pen_client = self.create_client(SetPen, f'/{name}/set_pen')
        
        if not pen_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'SetPen service for {name} not available, skipping pen color')
            return False
            
        pen_request = SetPen.Request()
        pen_request.r = color[0]
        pen_request.g = color[1]
        pen_request.b = color[2]
        pen_request.width = 3
        pen_request.off = 0
        
        future = pen_client.call_async(pen_request)
        future.add_done_callback(lambda future, n=name: self.pen_set_callback(future, n))
        return True
            
    def pen_set_callback(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(f'Set pen color for {name}')
            if name in self.boids:
                self.boids[name].pen_set = True
        except Exception as e:
            self.get_logger().warn(f'Failed to set pen color for {name}: {e}')
            
    def pose_callback(self, msg, name):
        if name in self.boids:
            self.boids[name].update_pose(msg)
            
            # set pen color on first pose received (when turtle is fully created)
            if not self.boids[name].pen_set:
                self.set_pen_color(name)
            
    def update_boids(self):
        # update if we have at least 2 turtles
        if len(self.boids) < 2:
            return
            
        for name, boid in self.boids.items():
            neighbors = self.get_neighbors(name)
            
            # apply boids rules (separate- avoid crowding nbrs and such; align with others avg dir; move towards the avg pos of neighbours)
            separation = self.apply_separation(boid, neighbors)
            alignment = self.apply_alignment(boid, neighbors)
            cohesion = self.apply_cohesion(boid, neighbors)
            boundary = self.apply_boundary_avoidance(boid)
            
            dx = (separation[0] * self.separation_weight + 
                  alignment[0] * self.alignment_weight + 
                  cohesion[0] * self.cohesion_weight +
                  boundary[0] * self.boundary_weight)
            
            dy = (separation[1] * self.separation_weight + 
                  alignment[1] * self.alignment_weight + 
                  cohesion[1] * self.cohesion_weight +
                  boundary[1] * self.boundary_weight)
            
            desired_velocity = math.sqrt(dx**2 + dy**2)
            desired_velocity = min(desired_velocity, self.max_speed)
            
            if desired_velocity > 0.1:  # change direction if we're moving
                desired_theta = math.atan2(dy, dx)
            else:
                desired_theta = boid.theta  
            
            self.publish_velocity(name, desired_velocity, desired_theta)
                
    def get_neighbors(self, name):
        current_boid = self.boids[name]
        neighbors = []
        
        for other_name, other_boid in self.boids.items():
            if other_name != name:
                distance = math.sqrt((current_boid.x - other_boid.x)**2 + 
                                    (current_boid.y - other_boid.y)**2)
                if distance < self.neighbor_distance:
                    neighbors.append(other_boid)
                    
        return neighbors
        
    def apply_separation(self, boid, neighbors):
        # avoid crowding neighbors
        close_dx, close_dy = 0.0, 0.0
        
        for neighbor in neighbors:
            distance = math.sqrt((boid.x - neighbor.x)**2 + (boid.y - neighbor.y)**2)
            if distance < self.neighbor_distance / 2 and distance > 0.1:  # don't divide by zero
                strength = 1.0 / distance  # stronger force applied for closer neighbors
                close_dx += (boid.x - neighbor.x) * strength
                close_dy += (boid.y - neighbor.y) * strength
                
        return close_dx, close_dy
        
    def apply_alignment(self, boid, neighbors):
        # go towards the average dir of neighbors
        avg_dx, avg_dy = 0.0, 0.0
        
        if neighbors:
            for neighbor in neighbors:
                avg_dx += math.cos(neighbor.theta)
                avg_dy += math.sin(neighbor.theta)
                
            avg_dx /= len(neighbors)
            avg_dy /= len(neighbors)
            
        return avg_dx, avg_dy
        
    def apply_cohesion(self, boid, neighbors):
        # move toward the average position of neighbors
        center_x, center_y = 0.0, 0.0
        
        if neighbors:
            for neighbor in neighbors:
                center_x += neighbor.x
                center_y += neighbor.y
                
            center_x /= len(neighbors)
            center_y /= len(neighbors)
            return center_x - boid.x, center_y - boid.y
        
        return 0.0, 0.0
            
    def apply_boundary_avoidance(self, boid):
        # go away from boundaries (otherwise they sort of just wiggle around there)
        boundary_dx, boundary_dy = 0.0, 0.0
        
        # boundaries are approximately 0 to 11 for x and y

        # left wall
        if boid.x < self.boundary_margin:
            boundary_dx += 1.0 / (boid.x + 0.1)  # stronger force shd be applied as we get closer to wall
            
        # right wall
        if boid.x > 11.0 - self.boundary_margin:
            boundary_dx -= 1.0 / (11.1 - boid.x)
            
        # bottom wall
        if boid.y < self.boundary_margin:
            boundary_dy += 1.0 / (boid.y + 0.1)
            
        # top wall
        if boid.y > 11.0 - self.boundary_margin:
            boundary_dy -= 1.0 / (11.1 - boid.y)
            
        return boundary_dx, boundary_dy
        
    def publish_velocity(self, name, linear_vel, angular_pos):
        if name not in self.cmd_vel_publishers:
            return
            
        twist = Twist()
        twist.linear.x = linear_vel
        
        current_boid = self.boids[name]
        angle_diff = angular_pos - current_boid.theta
        
        # normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        twist.angular.z = angle_diff * 2.0  
        
        self.cmd_vel_publishers[name].publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BoidsSwarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
