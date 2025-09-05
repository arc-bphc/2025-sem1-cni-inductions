import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class BoidController(Node):
    def __init__(self, turtle_names):
        super().__init__('boid_controller')

        # Avoid clash with Node internals: use a unique name
        self.turtle_publishers = {
            name: self.create_publisher(Twist, f'{name}/cmd_vel', 10)
            for name in turtle_names
        }

        # Track each turtle's current pose
        self.poses = {name: None for name in turtle_names}

        # Subscribe to each turtle's pose topic
        for name in turtle_names:
            self.create_subscription(
                Pose,
                f'{name}/pose',
                lambda msg, n=name: self.pose_callback(msg, n),
                10
            )

        # Timer to call update at 10 Hz
        self.timer = self.create_timer(0.1, self.update)

    def pose_callback(self, msg, turtle_name):
        self.poses[turtle_name] = msg

    def update(self):
        # Wait until we have poses for everyone
        if any(pose is None for pose in self.poses.values()):
            return

        names = list(self.poses.keys())

        for i, name in enumerate(names):
            me = self.poses[name]
            # Compute average position of others
            avg_x = 0.0
            avg_y = 0.0
            count = 0
            for j, other_name in enumerate(names):
                if i == j:
                    continue
                other = self.poses[other_name]
                avg_x += other.x
                avg_y += other.y
                count += 1
            if count > 0:
                avg_x /= count
                avg_y /= count
            else:
                avg_x = me.x
                avg_y = me.y

            # Vector towards centre of mass
            dx = avg_x - me.x
            dy = avg_y - me.y
            distance = math.hypot(dx, dy)

            # Simple avoidance: if too close to average, steer away
            if distance < 1.0:
                dx = -dx
                dy = -dy

            # Compute desired heading
            target_theta = math.atan2(dy, dx)
            heading_error = target_theta - me.theta

            # Normalise heading to [-pi, pi]
            while heading_error > math.pi:
                heading_error -= 2.0 * math.pi
            while heading_error < -math.pi:
                heading_error += 2.0 * math.pi

            twist = Twist()
            twist.linear.x = 1.5   # constant forward speed
            twist.angular.z = 2.0 * heading_error  # steer towards/away from average

            self.turtle_publishers[name].publish(twist)


def main():
    rclpy.init()
    # Adjust this list to match the turtles you spawn
    turtle_names = ['turtle1', 'turtle2', 'turtle3']
    node = BoidController(turtle_names)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

