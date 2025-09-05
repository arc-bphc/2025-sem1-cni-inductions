import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
import math


class BoidsNode(Node):
    def __init__(self):
        super().__init__('boids_node')

        #params
        self.num_boids = 8
        self.neighbor_dist = 7.0
        self.max_linear = 2.0
        self.max_angular = 3.0
        self.sep_weight = 1.0
        self.align_weight = 0.75
        self.coh_weight = 3.0
        self.bound_weight = 2.0

        self.names = []
        self.poses = {}
        self.pubs = {}

        #clear prev run
        self.kill_client = self.create_client(Kill, 'kill')
        for name in ['turtle1'] + [f'boid{i+1}' for i in range(50)]:
            req = Kill.Request()
            req.name = name
            future = self.kill_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

        #spawn
        self.spawn_client = self.create_client(Spawn, 'spawn')
        for i in range(self.num_boids):
            req = Spawn.Request()
            req.x = random.uniform(2.0, 9.0)
            req.y = random.uniform(2.0, 9.0)
            req.theta = random.uniform(-math.pi, math.pi)
            req.name = f'boid{i+1}'

            future = self.spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            try:
                resp = future.result()
                name = resp.name
            except Exception as e:
                self.get_logger().error(f'Failed to spawn boid {i+1}: {e}')
                name = req.name

            self.names.append(name)
            self.pubs[name] = self.create_publisher(Twist, f'{name}/cmd_vel', 10)
            self.create_subscription(Pose, f'{name}/pose', self.make_pose_callback(name), 10)

        self.timer = self.create_timer(0.1, self.update)

    def make_pose_callback(self, name):
        def callback(msg: Pose):
            self.poses[name] = msg
        return callback

    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def update(self):
        if len(self.poses) < len(self.names):
            return

        for name in self.names:
            boid = self.poses[name]

            sep_x = sep_y = 0.0
            ali_x = ali_y = 0.0
            coh_x = coh_y = 0.0
            count = 0

            for other_name, other in self.poses.items():
                if name == other_name:
                    continue
                d = self.distance(boid, other)
                if d < self.neighbor_dist and d > 0:

                    #separation
                    if d < self.neighbor_dist * 0.5:
                        sep_x += (boid.x - other.x) / (d**2)
                        sep_y += (boid.y - other.y) / (d**2)

                    #alignment
                    ali_x += math.cos(other.theta)
                    ali_y += math.sin(other.theta)

                    #cohesion
                    coh_x += other.x
                    coh_y += other.y
                    count += 1

            if count > 0:
                ali_x /= count
                ali_y /= count
                coh_x = (coh_x / count) - boid.x
                coh_y = (coh_y / count) - boid.y

            steer_x = (self.sep_weight * sep_x +
                       self.align_weight * ali_x +
                       self.coh_weight * coh_x)
            steer_y = (self.sep_weight * sep_y +
                       self.align_weight * ali_y +
                       self.coh_weight * coh_y)

            #avoid map edge
            margin = 1.0
            xmin, xmax, ymin, ymax = 0.5, 10.5, 0.5, 10.5
            bx = by = 0.0
            if boid.x < xmin + margin:
                bx += (xmin + margin - boid.x)
            elif boid.x > xmax - margin:
                bx -= (boid.x - (xmax - margin))
            if boid.y < ymin + margin:
                by += (ymin + margin - boid.y)
            elif boid.y > ymax - margin:
                by -= (boid.y - (ymax - margin))

            steer_x += 2.0 * bx
            steer_y += 2.0 * by

            steer_x += random.uniform(-0.2, 0.2)
            steer_y += random.uniform(-0.2, 0.2)

            angle = math.atan2(steer_y, steer_x)
            speed = math.sqrt(steer_x ** 2 + steer_y ** 2)

            #publish
            twist = Twist()
            twist.linear.x = min(speed, self.max_linear)
            diff = (angle - boid.theta + math.pi) % (2 * math.pi) - math.pi
            twist.angular.z = max(-self.max_angular, min(self.max_angular, diff))
            self.pubs[name].publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BoidsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
