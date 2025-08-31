import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random

NUM_TURTLES = 5
MAX_SPEED = 2.0
MAX_TURN = 2.0

# Boids parameters (tweak these to change behavior)
PERCEPTION_RADIUS = 3.0
SEPARATION_DISTANCE = 0.8
COHESION_WEIGHT = 0.02
SEPARATION_WEIGHT = 0.5
ALIGNMENT_WEIGHT = 0.15

class Turtle:
    def __init__(self, name):
        self.name = name
        self.pose = Pose()

class BoidsControllerNode(Node):
    def __init__(self):
        super().__init__('boids_controller')
        self.turtles = {}
        self.vel_publishers = {}  # Renamed this variable

        # Spawn turtles
        self.spawn_turtles()

        # Create a timer to run the Boids update loop
        self.timer = self.create_timer(0.1, self.update_boids)

    def spawn_turtles(self):
        # Add the default turtle1
        self.turtles['turtle1'] = Turtle('turtle1')

        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')

        for i in range(2, NUM_TURTLES + 1):
            name = f'turtle{i}'
            req = Spawn.Request()
            req.x = random.uniform(1.0, 10.0)
            req.y = random.uniform(1.0, 10.0)
            req.theta = random.uniform(0.0, 2 * math.pi)
            req.name = name

            future = spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.turtles[name] = Turtle(name)
                self.get_logger().info(f'Successfully spawned {name}')
            else:
                self.get_logger().error(f'Failed to spawn {name}')

        # After spawning, create all publishers and subscribers
        for name in self.turtles.keys():
            # Use the new variable name here
            self.vel_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(
                Pose,
                f'/{name}/pose',
                lambda msg, turtle_name=name: self.pose_callback(msg, turtle_name),
                10)

    def pose_callback(self, msg, turtle_name):
        if turtle_name in self.turtles:
            self.turtles[turtle_name].pose = msg

    def update_boids(self):
        for name, turtle in self.turtles.items():

            v_cohesion = self.calculate_cohesion(turtle)
            v_separation = self.calculate_separation(turtle)
            v_alignment = self.calculate_alignment(turtle)

            # Combine the vectors with weights
            final_vector_x = (v_cohesion[0] * COHESION_WEIGHT +
                              v_separation[0] * SEPARATION_WEIGHT +
                              v_alignment[0] * ALIGNMENT_WEIGHT)
            final_vector_y = (v_cohesion[1] * COHESION_WEIGHT +
                              v_separation[1] * SEPARATION_WEIGHT +
                              v_alignment[1] * ALIGNMENT_WEIGHT)

            # Create Twist message from the final vector
            twist = Twist()
            goal_angle = math.atan2(final_vector_y, final_vector_x)
            angle_diff = self.normalize_angle(goal_angle - turtle.pose.theta)

            twist.angular.z = max(-MAX_TURN, min(MAX_TURN, angle_diff * 2.0))
            twist.linear.x = MAX_SPEED

            # And use the new variable name here
            self.vel_publishers[name].publish(twist)

    def get_neighbors(self, current_turtle):
        neighbors = []
        for name, other_turtle in self.turtles.items():
            if current_turtle.name == name:
                continue
            dist = math.sqrt((current_turtle.pose.x - other_turtle.pose.x)**2 + 
                             (current_turtle.pose.y - other_turtle.pose.y)**2)
            if dist < PERCEPTION_RADIUS:
                neighbors.append(other_turtle)
        return neighbors

    def calculate_cohesion(self, turtle):
        neighbors = self.get_neighbors(turtle)
        if not neighbors:
            return (0, 0)

        avg_x = sum(n.pose.x for n in neighbors) / len(neighbors)
        avg_y = sum(n.pose.y for n in neighbors) / len(neighbors)

        return (avg_x - turtle.pose.x, avg_y - turtle.pose.y)

    def calculate_separation(self, turtle):
        sep_vector = [0, 0]
        for name, other_turtle in self.turtles.items():
            if turtle.name == name:
                continue
            dist = math.sqrt((turtle.pose.x - other_turtle.pose.x)**2 + 
                             (turtle.pose.y - other_turtle.pose.y)**2)
            if dist < SEPARATION_DISTANCE:
                sep_vector[0] += turtle.pose.x - other_turtle.pose.x
                sep_vector[1] += turtle.pose.y - other_turtle.pose.y
        return tuple(sep_vector)

    def calculate_alignment(self, turtle):
        neighbors = self.get_neighbors(turtle)
        if not neighbors:
            return (0, 0)

        avg_vx = sum(math.cos(n.pose.theta) for n in neighbors) / len(neighbors)
        avg_vy = sum(math.sin(n.pose.theta) for n in neighbors) / len(neighbors)

        return (avg_vx, avg_vy)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = BoidsControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()