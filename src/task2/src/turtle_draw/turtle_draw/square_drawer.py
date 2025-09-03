import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()

    node = Node("square_drawer")
    publisher = node.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    msg = Twist()
    node.get_logger().info("Turtle is drawing a square...")

    while rclpy.ok():
        # move forward
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        for _ in range(20):   # small loop to keep moving
            publisher.publish(msg)
            time.sleep(0.1)

        # turn 90 degrees
        msg.linear.x = 0.0
        msg.angular.z = 1.57  # ~90 degrees per second
        for _ in range(10):   # small loop to keep turning
            publisher.publish(msg)
            time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
