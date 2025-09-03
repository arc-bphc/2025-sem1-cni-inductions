import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
import math



class MinimalTurtleNode(Node):
    def __init__(self):
        super().__init__('minimal_turtle_node')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')
        
        self.pose = None
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.points = None
        self.current_pt = 0
        self.timer = self.create_timer(1.0, self.publish_cmd)

    def circle_pts(self, r):
        if self.pose is None: return []
        points = []
        for i in range(72):
            x = r * math.cos(5 * i * (2*math.pi) / 360)
            y = r * math.sin(5 * i * (2*math.pi) / 360)
            points.append([self.pose.x + x, self.pose.y + y])
        return points

    def publish_cmd(self):
        if self.pose is None:
            return
        if self.current_pt == 0: self.call_set_pen(1)
        elif self.current_pt == 1: self.call_set_pen(0)

        x1, y1 = self.pose.x, self.pose.y
        x2, y2 = self.points[self.current_pt]

        msg = Twist()
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx**2 + dy**2)

        target_theta = math.atan2(dy, dx)
        angle_error = target_theta - self.pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error)) 

        if abs(angle_error) > 0.1:
            msg.angular.z = angle_error
            msg.linear.x = 0.0
        else:
            msg.linear.x = distance
            msg.angular.z = 0.0

        self.publisher.publish(msg)
        if distance < 0.1:
            self.current_pt = (self.current_pt + 1)
        if self.current_pt >= len(self.points): 
            self.get_logger().info("Finished!")
            self.timer.cancel()
            raise SystemExit
            return

    def pen_response(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Pen service failed: {e}")

    def call_set_pen(self, off: int):
        """off=1 → pen up, off=0 → pen down"""
        req = SetPen.Request()
        req.r, req.g, req.b, req.width, req.off = (0, 0, 0, 2, off)
        future = self.pen_client.call_async(req)
        future.add_done_callback(self.pen_response)

    def pose_callback(self, msg: Pose):
        self.pose = msg
        if self.points is None:
            self.points = self.circle_pts(.5)
    

def main(args=None):
    rclpy.init(args=args)
    node = MinimalTurtleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
