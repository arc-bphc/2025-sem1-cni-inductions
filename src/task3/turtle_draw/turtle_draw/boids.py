import rclpy      # main python library for ros2
from rclpy.node import Node    # imports Node class
from turtlesim.srv import Spawn   # spawning turtles
from turtlesim.msg import Pose   # tracking positions
from geometry_msgs.msg import Twist  # used for velocities
import math    # operations
import random     # randomization 
from rclpy.executors import MultiThreadedExecutor    # lets my ros2 program handle multiple turtles at once
 

class Vector:
    def __init__(self, x=0.0, y=0.0): 
        self.x = x
        self.y = y

    def __add__(self, other):                             # operations
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar): 
        return Vector(self.x / scalar, self.y / scalar)

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)    

    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector(0.0, 0.0)
        return self / mag

    def limit(self, max_val):  
        mag = self.magnitude()
        if mag > max_val:
            return self.normalize() * max_val
        return self


class BoidNode(Node):
    def __init__(self, name, all_names):
        super().__init__('boid_' + name)
        self.name = name                  # stores this turtles names
        self.position = Vector()          # gives boids current position as 0,0
        self.velocity = Vector(random.uniform(-8, 8), random.uniform(-8, 8))    # gives random initial velocity
        self.theta = 0.0
        self.all_names = all_names        # stores list of all turtles
 
        # listens to published pose and creates specific turtle wise positions
        self.pose_sub = self.create_subscription(Pose, f'/{name}/pose', self.pose_callback, 10)  
        
        # creates publisher which publishes ' twist ' type messages for commanding velocity
        self.pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)   

        # creates a dataset of neighbor positions excluding itself ( turt1 ) 
        self.neighbor_positions = {n: None for n in all_names if n != name}     
        
        # Subscribe to each neighbor's pose topic to receive their positions
        for n in self.neighbor_positions:
            self.create_subscription(Pose, f'/{n}/pose', self.make_neighbor_pose_cb(n), 10) 

        # Timer to call update every 50 milli secs to update turt behavior
        self.timer = self.create_timer(0.05, self.update)  

    def pose_callback(self, msg):            # callback for pose updates
        self.position = Vector(msg.x, msg.y)   # update position vector
        self.theta = msg.theta                   # update angle

    def make_neighbor_pose_cb(self, neighbor_name):     # creates a callback for neighbor pose updates
        def callback(msg):
            self.neighbor_positions[neighbor_name] = Vector(msg.x, msg.y)
        return callback

    def separation(self, neighbors, desired_separation=1.0):      # calculates separation vector from neighbors
        steer = Vector()
        count = 0
        for pos in neighbors:
            d = (self.position - pos).magnitude()
            if 0 < d < desired_separation:                   
                steer += (self.position - pos).normalize() / d
                count += 1
        if count > 0:
            steer = steer / count              # averages the steering vector
        return steer

    def alignment(self, boids):                # rule to align velocity with nearby boids
        avg_vel = Vector()
        count = 0
        for b in boids:
            if b.name != self.name:
                avg_vel += b.velocity
                count += 1
        if count > 0:
            avg_vel = avg_vel / count
            return (avg_vel - self.velocity) * 0.1      # small adjustment towards average velocity
        return Vector()

    def cohesion(self, neighbors):              # rule to move toward center of mass of neighbors
        center = Vector()
        count = 0
        for pos in neighbors:
            center += pos
            count += 1
        if count > 0:
            center = center / count
            return (center - self.position) * 0.01  # small pull towards center
        return Vector()

    def update(self):        # periodically updates itself 
       
        if self.position is None or any(n is None for n in self.neighbor_positions.values()):
            return          # wait until turt and neighbors positions are found


        neighbor_pos_list = list(self.neighbor_positions.values())

        # Calculate separation and cohesion steering vectors
        v_sep = self.separation(neighbor_pos_list)
        v_coh = self.cohesion(neighbor_pos_list)
        
        # Update velocity
        self.velocity += v_sep * 1.5 + v_coh * 1.0

        # Limit max velocity magnitude to 2
        self.velocity = self.velocity.limit(2.0)

        # Calculate desired angle from velocity vector
        desired_theta = math.atan2(self.velocity.y, self.velocity.x)
        angle_diff = desired_theta - self.theta

         # Normalize angle difference between -pi and pi
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi


        # create twist message
        twist = Twist()
        twist.linear.x = self.velocity.magnitude()
        twist.angular.z = 3.0 * angle_diff

        # publish the twist message to command turt
        self.pub.publish(twist)


class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')
        self.turtles = ['turtle1', 'turtle2', 'turtle3', 'turtle4', 'turtle5']

        # client to spawn turtles
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # wait until spawn service available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # spawn turtles at random positions
        for i in range(2, 6):
            req = Spawn.Request()
            req.x = random.uniform(2, 9)
            req.y = random.uniform(2, 9)
            req.theta = 0.0
            req.name = f'turtle{i}'
            self.spawn_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    swarm_manager = SwarmManager()

    # create boid nodes for each turtle
    boid_nodes = [BoidNode(name, ['turtle1', 'turtle2', 'turtle3', 'turtle4', 'turtle5']) for name in ['turtle1', 'turtle2', 'turtle3', 'turtle4', 'turtle5']]

    # executor to run swarm_manager
    executor = MultiThreadedExecutor()
    executor.add_node(swarm_manager)
    for node in boid_nodes:
        executor.add_node(node)

    try:
        executor.spin()     # # keep running until interrupted
    except KeyboardInterrupt:
    
        pass
    finally:
        # clear nodes and shutdown ros2 properly
        swarm_manager.destroy_node()
        for node in boid_nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
