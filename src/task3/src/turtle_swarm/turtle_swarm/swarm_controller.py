import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import numpy as np

# A constant for the number of turtles we expect
TURTLE_COUNT = 5

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        self.get_logger().info("Boids swarm controller has been started.")

        # --- Parameters for Boids Algorithm ---
        self.COHESION_WEIGHT = 1.0       # How much to steer towards the center of the flock
        self.SEPARATION_WEIGHT = 2.0     # How much to steer to avoid crowding
        self.ALIGNMENT_WEIGHT = 0.5      # How much to steer to align with the flock's direction
        
        self.NEIGHBOR_RADIUS = 25.0      # The distance to consider a turtle a "neighbor"
        self.SEPARATION_RADIUS = 1.5    # The minimum distance to keep from other turtles
        
        self.MAX_SPEED = 2.0             # Maximum linear speed for a turtle
        self.MAX_TURN = 3.0              # Maximum angular speed (turning speed)
        
        # Dictionary to store the current pose of each turtle
        self.turtle_poses = {}

        # List to store our velocity publishers
        self.velocity_publishers = []

        # Create subscribers and publishers for all turtles
        for i in range(1, TURTLE_COUNT + 1):
            turtle_name = f'turtle{i}'
            self.create_subscription(Pose, f'/{turtle_name}/pose', lambda msg, name=turtle_name: self.pose_callback(msg, name), 10)
            publisher = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
            self.velocity_publishers.append(publisher)
            
        # Create a timer that will call the move_turtles function periodically
        self.timer = self.create_timer(0.1, self.move_turtles) # Run at 10 Hz

    def pose_callback(self, msg, turtle_name):
        self.turtle_poses[turtle_name] = msg
        
    def move_turtles(self):
        if len(self.turtle_poses) < TURTLE_COUNT:
            self.get_logger().warn("Waiting for all turtles to be spawned...")
            return

        for i in range(TURTLE_COUNT):
            current_turtle_name = f'turtle{i+1}'
            current_pose = self.turtle_poses.get(current_turtle_name)
            
            if not current_pose:
                continue

            # --- Initialize Boids Vectors ---
            cohesion_vector = np.array([0.0, 0.0])
            separation_vector = np.array([0.0, 0.0])
            alignment_vector = np.array([0.0, 0.0])
            neighbor_count = 0

            # --- Calculate Boids rules based on neighbors ---
            for j in range(TURTLE_COUNT):
                if i == j:
                    continue

                other_turtle_name = f'turtle{j+1}'
                other_pose = self.turtle_poses.get(other_turtle_name)

                if not other_pose:
                    continue
                
                distance = math.sqrt((current_pose.x - other_pose.x)**2 + (current_pose.y - other_pose.y)**2)
                
                if distance < self.NEIGHBOR_RADIUS:
                    neighbor_count += 1
                    # Cohesion: Add other turtle's position to find the center later
                    cohesion_vector += np.array([other_pose.x, other_pose.y])
                    # Alignment: Add other turtle's direction vector
                    alignment_vector += np.array([math.cos(other_pose.theta), math.sin(other_pose.theta)])
                    # Separation: If too close, create a vector to steer away
                    if distance < self.SEPARATION_RADIUS:
                        separation_vector -= np.array([other_pose.x - current_pose.x, other_pose.y - current_pose.y])

            # --- Finalize Boids Calculations ---
            target_vector = np.array([0.0, 0.0])
            if neighbor_count > 0:
                # Cohesion: Steer towards the average position (center) of neighbors
                cohesion_vector = (cohesion_vector / neighbor_count) - np.array([current_pose.x, current_pose.y])
                # Alignment: Steer towards the average heading of neighbors
                alignment_vector = alignment_vector / neighbor_count
                
                # Combine vectors with weights
                target_vector += cohesion_vector * self.COHESION_WEIGHT
                target_vector += separation_vector * self.SEPARATION_WEIGHT
                target_vector += alignment_vector * self.ALIGNMENT_WEIGHT
            
            # --- Wall Avoidance (Higher Priority) ---
            wall_avoidance_vector = np.array([0.0, 0.0])
            margin = 1.0
            if current_pose.x < margin: wall_avoidance_vector[0] = 1.0
            if current_pose.x > 11.0 - margin: wall_avoidance_vector[0] = -1.0
            if current_pose.y < margin: wall_avoidance_vector[1] = 1.0
            if current_pose.y > 11.0 - margin: wall_avoidance_vector[1] = -1.0

            if not np.all(wall_avoidance_vector == 0):
                target_vector = wall_avoidance_vector
            
            # --- Calculate final velocity from the target vector ---
            twist_msg = Twist()
            if np.all(target_vector == 0):
                # If no neighbors and not near a wall, just move forward
                target_vector = np.array([math.cos(current_pose.theta), math.sin(current_pose.theta)])
            
            # Calculate desired angle and how much we need to turn
            desired_angle = math.atan2(target_vector[1], target_vector[0])
            angle_error = desired_angle - current_pose.theta

            # Normalize angle error to be between -pi and pi
            while angle_error > math.pi: angle_error -= 2 * math.pi
            while angle_error < -math.pi: angle_error += 2 * math.pi

            twist_msg.linear.x = self.MAX_SPEED
            twist_msg.angular.z = max(-self.MAX_TURN, min(self.MAX_TURN, 4.0 * angle_error)) # P-controller for turning
            
            # Publish the command
            self.velocity_publishers[i].publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    controller_node = SwarmController()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()