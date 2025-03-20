#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self, speed=0.2, duration=15.0):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed 
        self.duration = duration


    def move(self, direction='forward'):
        direction_multiplier = 1.1 if direction == 'forward' else -1
        print(f"Starting to move {direction}...")

        twist = Twist()
        twist.linear.x = self.speed * direction_multiplier

        start_time = time.time()
        elapsed_time = 0.0

        while elapsed_time < self.duration:
            self.cmd_vel_publisher.publish(twist)
            elapsed_time = time.time() - start_time

            traveled_distance = abs(self.speed) * elapsed_time
            print(f"Moved: {traveled_distance:.2f} meters", end='\r')

            time.sleep(0.1)

        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        print(f"\nFinished moving {direction}. Total distance: {traveled_distance:.2f} meters.")

        time.sleep(1.0)


def main():
    rclpy.init()

    node = TurtleBotController(speed=0.2, duration=15.0)

    node.move(direction='forward')
    node.move(direction='backward')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



