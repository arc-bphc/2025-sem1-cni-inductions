import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
import math


class BoidsSwarm(Node):
    def __init__(self, n_turtles=5, neighbor_radius=2.0):
        super().__init__('boids_swarm')
        self.turtle_names = ['turtle1'] 
        self.neighbor_radius = neighbor_radius

        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn...")
        self.freq = 0.1
        for i in range(2, n_turtles + 1):
            name = f'turtle{i}'
            x = random.uniform(1.0, 10.0)
            y = random.uniform(1.0, 10.0)
            theta = random.uniform(0, 2*math.pi)
            self.spawn_turtle(x, y, theta, name)
            self.turtle_names.append(name)

        self.pubs = {}
        self.poses = {}
        for name in self.turtle_names:
            self.pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(Pose, f'/{name}/pose',
                                     lambda msg, n=name: self.pose_callback(msg, n), 10)

        self.timer = self.create_timer(self.freq, self.update_swarm)

    def spawn_turtle(self, x, y, theta, name):
        req = Spawn.Request()
        req.x, req.y, req.theta, req.name = x, y, theta, name
        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(f"Spawned {f.result().name}"))

    def pose_callback(self, msg, name):
        self.poses[name] = msg

    def update_swarm(self):
        if len(self.poses) < len(self.turtle_names):
            return

        for name in self.turtle_names:
            me = self.poses[name]
            neighbors = [self.poses[n] for n in self.turtle_names
                         if n != name and self.dist(me, self.poses[n]) < self.neighbor_radius]
            
            vx, vy = 0.5, 0.5

            if neighbors:

                cx = sum(n.x for n in neighbors) / len(neighbors)
                cy = sum(n.y for n in neighbors) / len(neighbors)

                sx, sy = 0.0, 0.0
                for n in neighbors:
                    dx = me.x - n.x
                    dy = me.y - n.y
                    d = math.sqrt(dx*dx + dy*dy)
                    if d < 5.0:
                        sx += dx / (d + 10e-6)
                        sy += dy / (d + 10e-6)

                ax = sum(math.cos(n.theta) for n in neighbors) / len(neighbors)
                ay = sum(math.sin(n.theta) for n in neighbors) / len(neighbors)

                vx = 1.0 * (3.3*(cx - me.x) + 1.5*sx + 1.0*ax)
                vy = 1.0 * (3.3*(cy - me.y) + 1.5*sy + 1.0*ay)

            margin = 1.5 
            if me.x < margin and vx < 0: 
                vx = -vx
            if me.x > 11 - margin and vx > 0:  
                vx = -vx
            if me.y < margin and vy < 0:   
                vy = -vy
            if me.y > 11 - margin and vy > 0: 
                vy = -vy

            target_theta = math.atan2(vy, vx)
            angle_error = target_theta - me.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            msg = Twist()
            msg.angular.z = 2.0 * angle_error
            msg.linear.x = min(2.0 * math.sqrt(vx*vx + vy*vy), 2.5)
            self.pubs[name].publish(msg)

    def dist(self, a, b):
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


def main(args=None):
    rclpy.init(args=args)
    node = BoidsSwarm(n_turtles=6, neighbor_radius=3.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
