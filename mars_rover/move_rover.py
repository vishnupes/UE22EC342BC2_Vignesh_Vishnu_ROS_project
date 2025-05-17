#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class RoverMover(Node):
    def __init__(self):
        super().__init__('rover_mover')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_rover)
        self.start_time = time.time()
        self.get_logger().info('Mars Rover movement node has started!')
        
    def move_rover(self):
        # Calculate elapsed time
        elapsed = time.time() - self.start_time
        
        # Create a figure-8 pattern movement
        msg = Twist()
        
        # Simple figure-8 pattern
        cycle = elapsed % 20  # 20-second cycle
        
        if cycle < 5:  # Forward
            msg.linear.x = 0.3
            msg.angular.z = 0.0
            self.get_logger().info('Moving forward')
        elif cycle < 10:  # Turn left
            msg.linear.x = 0.2
            msg.angular.z = 0.5
            self.get_logger().info('Turning left')
        elif cycle < 15:  # Forward
            msg.linear.x = 0.3
            msg.angular.z = 0.0
            self.get_logger().info('Moving forward')
        else:  # Turn right
            msg.linear.x = 0.2
            msg.angular.z = -0.5
            self.get_logger().info('Turning right')
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoverMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Create a stop message before shutting down
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.get_logger().info('Stopping rover and shutting down')
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()