import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class TurtleSpawner(Node):
    def __init__(self):
        # Initialize the node with the name 'turtle_spawner'
        super().__init__('turtle_spawner')
        self.get_logger().info("Turtle spawner node has been started.")
        
        # A list to hold the names of our spawned turtles
        self.turtle_names = []

        # The turtle count, including the default one
        self.turtle_count = 5 
        
        # Start the spawning process
        self.spawn_turtles()

    def spawn_turtles(self):
        # Create a client to call the '/spawn' service
        # The service won't be available until turtlesim_node is running
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('The /spawn service is not available, waiting again...')

        # Spawn the desired number of turtles (we start from 2 since turtle1 already exists)
        for i in range(2, self.turtle_count + 1):
            turtle_name = f'turtle{i}'
            x = random.uniform(1.0, 10.0) # Random x position between 1.0 and 10.0
            y = random.uniform(1.0, 10.0) # Random y position between 1.0 and 10.0
            theta = random.uniform(0.0, 2 * 3.14159) # Random orientation in radians
            
            # Create the request to send to the service
            request = Spawn.Request()
            request.x = x
            request.y = y
            request.theta = theta
            request.name = turtle_name

            # Call the service and wait for the result
            future = spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            try:
                response = future.result()
                if response:
                    self.turtle_names.append(response.name)
                    self.get_logger().info(f'Successfully spawned turtle: {response.name}')
            except Exception as e:
                self.get_logger().error(f'Service call failed for spawning {turtle_name}: {e}')

        self.get_logger().info(f"All {self.turtle_count} turtles are now in the pond!")
        
        # We can shutdown the node now that its job is done.
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of our spawner node
    spawner_node = TurtleSpawner()
    
    # rclpy.spin() is not needed here because the node shuts itself down 
    # after spawning is complete.
    
    # The shutdown call is in the node's __init__, so we are done.

if __name__ == '__main__':
    main()