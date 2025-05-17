#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time  # Added time import at the top level

msg = """
Mars Rover Teleop Control
---------------------------
Moving around:
   w    
a  s  d    q
   x    

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s   : reverse (go backwards)
q   : brake (stop without reversing)
CTRL-C to quit
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('teleop_rover')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.running = True
        
        # Create a separate thread for publishing at fixed rate
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        self.get_logger().info('Teleop node has started!')
        print(msg)
    
    def publish_loop(self):
        rate = 0.1  # seconds
        while self.running:
            self.publish_velocity()
            time.sleep(rate)
    
    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)
        
    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run_keyboard_teleop(self):
        while self.running:
            key = self.get_key()
            if key == '\x03':  # CTRL+C
                self.running = False
                break
                
            # Update velocities based on key press
            if key == 'w':
                self.linear_speed += 0.1
                print(f"Linear speed: {self.linear_speed:.1f}")
            elif key == 'x':
                self.linear_speed -= 0.1
                print(f"Linear speed: {self.linear_speed:.1f}")
            elif key == 'a':
                self.angular_speed += 0.1
                print(f"Angular speed: {self.angular_speed:.1f}")
            elif key == 'd':
                self.angular_speed -= 0.1
                print(f"Angular speed: {self.angular_speed:.1f}")
            elif key == 's':
                # Reverse/backwards
                self.linear_speed -= 0.1
                print(f"Linear speed: {self.linear_speed:.1f}")
            elif key == 'q':
                # Proper brake (stop without reversing)
                self.linear_speed = 0.0
                self.angular_speed -= 0.0
                print(f"Linear speed: {self.linear_speed:.1f}")

            # Ensure speeds are capped
            self.linear_speed = max(min(self.linear_speed, 1.0), -1.0)
            self.angular_speed = max(min(self.angular_speed, 1.0), -1.0)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        # Run keyboard control in main thread
        node.run_keyboard_teleop()
    except Exception as e:
        print(e)
    finally:
        # Stop the rover before exiting
        node.linear_speed = 0.0
        node.angular_speed = 0.0
        node.publish_velocity()
        time.sleep(0.1)  # Give time for the message to be published
        
        node.running = False
        if node.publish_thread.is_alive():
            node.publish_thread.join()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
