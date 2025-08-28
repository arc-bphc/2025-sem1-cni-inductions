"""
 *  author: realpratz [Pratyush Priyadarshi]
"""

import rclpy
from rclpy.node import Node
import math
import random
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

"""
- This was an attempt (this is basically an excerpt from the opensource numpy library) to create a simple 2D vector class, since array calculations were messy in nature unlike Task2 where it could be handeled.
- Intially I worked with methods directly from this class, but then came across "Dunder functions" which simplified the workflow a lot. (so, add became __add__)
"""
class Vector:
    def __init__(self, x=0, y=0): #defining what a vector object is
        self.x = x
        self.y = y

    def __add__(self, other): #addition of two vector objects (+)
        return Vector(self.x + other.x, self.y + other.y) 

    def __sub__(self, other): #subtraction of two vector objects (-)
        return Vector(self.x - other.x, self.y - other.y)

    def __iadd__(self, other): #self addition (+=)
        self.x += other.x
        self.y += other.y
        return self

    def __isub__(self, other): #self subtraction (-=)
        self.x -= other.x
        self.y -= other.y
        return self

    def __truediv__(self, scalar): #scalar division of a vector object (/)
        return Vector(self.x / scalar, self.y / scalar)

    def __mul__(self, scalar): #scalar multiplication of a vector object (*)
        return Vector(self.x * scalar, self.y * scalar)

    def magnitude(self): #getting the magnitude of a vector
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self): #normalizing a vector object, WITHOUT creating a new vector object
        mag = self.magnitude()
        if mag == 0:
            return Vector()
        return self / mag

    def limit(self, max_val): #limiting vector definition w.r.t magnitude
        mag = self.magnitude()
        if mag > max_val:
            return self.normalize() * max_val
        return self

"""
- This boids algorithm is based off Conrad Parker's Explanation and Logic along with an attempt to replicate Sebastian Lague's simulation
"""
class BoidTurtle:
    def __init__(self, name, node):
        self.name = name
        self.node = node 
        self.position = Vector(random.uniform(2, 9), random.uniform(2, 9)) #randomizing intial position
        self.velocity = Vector(random.uniform(-1, 1), random.uniform(-1, 1)) #randomizing intial velocity
        self.theta = 0.0 # where turtle facing
        self.subscriber_ = node.create_subscription(Pose, "/" + name + "/pose", self.pose_callback, 10) # Subscriber to get back turtle data
        self.publisher_ = node.create_publisher(Twist, "/" + name + "/cmd_vel", 10) # Publisher to control the turtles

    def pose_callback(self, msg):
        self.position = Vector(msg.x, msg.y)
        self.theta = msg.theta

    """
    Main three rules: Cohesion, Seperation, Alignment
    """
    #Cohesion: move toward the average position of neighbors.
    def cohesion(self, boids):
        center = Vector() #we're gonna find the "percieved" center of mass of the boid flock
        for b in boids:
            if b != self: #percieved means the COM of all boids except it itself
                center += b.position 
        center = center / (len(boids) - 1) #COM=sum(mixi)/sum(xi)
        return (center - self.position) / 10 #we move it about 10% closer of the distance between the percieved center and itself

    #Separation: avoid crowding neighbors.
    def seperation(self, boids, min_distance=1.0):
        c = Vector() #we're gonna find a vector which when added to the self position, makes it move away. we want to avoid crashing
        for b in boids:
            if b != self: #we don't want the boid to repel from itself lol
                if (b.position - self.position).magnitude() < min_distance: #if the boid is within the range of any other boid except itself, then c will be the difference between them
                    c -= (b.position - self.position)
        return c #this c will later be added to the position to repel it away

    #Alignment: move in the same direction as neighbors.
    def alignment(self, boids):
        pv = Vector() #we're gonna find the "percieved" velocity of the boid flock
        for b in boids:
            if b != self: #percieved means the velocity of all boids except it itself
                pv += b.velocity
        pv = pv / (len(boids) - 1) #COM=sum(mixi)/sum(xi)
        return (pv - self.velocity) / 5 #we add a 1/5th of the portion of this velocity to the boid
    
    """
    In the first few generations of tests, the turtles kept on moving in circles if they were too far (added: wander), and stuck at walls (added: bound_position)
    """
    #essentially creating a unit vector in the randomized direction then give it some magnitude using strength
    def wander(self):
        angle = random.uniform(-0.05, 0.05)
        strength = 0.05
        return Vector(math.cos(self.theta + angle), math.sin(self.theta + angle)) * strength
    
    #self explanatory; bounds the turtles (or atleast tries to, since the high velocities sometimes causes issues) to x:[1,10] and y:[1,10] by adding reversal in velocity
    def bound_position(self, xmin=1.0, xmax=10.0, ymin=1.0, ymax=10.0):
        v = Vector()
        if self.position.x < xmin:
            v.x = 0.5
        elif self.position.x > xmax:
            v.x = -0.5

        if self.position.y < ymin:
            v.y = 0.5
        elif self.position.y > ymax:
            v.y = -0.5

        return v

    """
    The actual combination of the 3+2 rules for movement, we also try to implement a function to keep facing velocity vector
    """
    def move(self, boids):
        v1 = self.cohesion(boids)
        v2 = self.seperation(boids)
        v3 = self.alignment(boids)
        v4 = self.wander()
        v5 = self.bound_position()

        #Each of the boids rules works independently, so, for each boid, you calculate how much it will get moved by each of the five rules, giving you five velocity vectors.
        self.velocity += v1 + v2 + v3 + v4 + v5

        # Real animals can't go infinitely fast, so limit boid velocity
        self.velocity = self.velocity.limit(4.0)

        # where I should be pointing if I align with my movement direction
        desired_theta = math.atan2(self.velocity.y, self.velocity.x) 

        # how much the boid needs to rotate to face the velocity vector
        angle_diff = desired_theta - self.theta

        # we want the shortest signed angle diff (beyond -pi and pi, it'll repeat)
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi #(if t>pi, then subtracting 2pi makes it between [-pi,pi])
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi #(if t<-pi, then adding 2pi makes it between [-pi,pi])

        msg = Twist()
        msg.angular.z = angle_diff * 2.0
        msg.linear.x = self.velocity.magnitude() * 2.0
        self.publisher_.publish(msg)

class swarm_boids(Node):
    def __init__(self):
        super().__init__('swarm_boids')
        self.boids = []
        self.spawner_ = self.create_client(Spawn, '/spawn') # Spawner to spawn turtles

        # turtle 1 is already spawned by default, we'll spawn 6 more
        for i in range(2, 8):
            request = Spawn.Request()
            request.x = random.uniform(2, 9)
            request.y = random.uniform(2, 9)
            request.theta = 0.0
            request.name = "turtle" + str(i)
            self.spawner_.call_async(request)

        # define turtles individually
        names = []
        for j in range(1, 8):
            names.append("turtle" + str(j))

        for n in names:
            self.boids.append(BoidTurtle(n, self))

        self.timer = self.create_timer(0.02, self.update_boids) # Timer to call publish_cmd every 0.02 second

    def update_boids(self):
        for b in self.boids:
            b.move(self.boids)

def main(args=None):
    rclpy.init(args=args)
    node = swarm_boids()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()