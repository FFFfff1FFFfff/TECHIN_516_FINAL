#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self, speed=0.2):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed

    def move(self, distance):
        direction = 1 if distance > 0 else -1
        distance = abs(distance)

        print(f"Starting to move {'forward' if direction == 1 else 'backward'} for {distance} meters...")

        twist = Twist()
        twist.linear.x = self.speed * direction

        start_time = time.time()
        elapsed_time = 0.0
        duration = distance / self.speed

        while elapsed_time < duration:
            self.cmd_vel_publisher.publish(twist)
            elapsed_time = time.time() - start_time
            traveled_distance = self.speed * elapsed_time * direction
            print(f"Moved: {traveled_distance:.2f} meters", end='\r')
            time.sleep(0.1)

        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        print(f"\nFinished moving {'forward' if direction == 1 else 'backward'}. Total distance: {distance} meters.")
        time.sleep(1.0)

    def turn(self, direction='left', angle=90):
        print(f"Starting to turn {direction} for {angle} degrees...")

        twist = Twist()
        if direction == 'left':
            twist.angular.z = 0.78
        elif direction == 'right':
            twist.angular.z = -0.78
        else:
            print(f"Invalid direction: {direction}")
            return

        turn_duration = (angle / 90) * 2.0
        start_time = time.time()
        elapsed_time = 0.0

        while elapsed_time < turn_duration:
            self.cmd_vel_publisher.publish(twist)
            elapsed_time = time.time() - start_time
            time.sleep(0.1)

        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        print(f"Finished turning {direction}.\n")
        time.sleep(1.0)

def main():
    rclpy.init()
    node = TurtleBotController(speed=0.2)
    
    # for
    node.move(-0.45)
    node.turn('right')
    node.move(-0.59)
    node.turn('left')
    node.move(-1.1)
    node.turn('left')
    node.move(-0.8)
    node.turn('right')
    node.move(-1.65)
    node.turn('right')
    node.move(-0.45)

    time.sleep(150.0)
    
    # back
    node.move(0.45)
    node.turn('left')
    node.move(1.65)
    node.turn('left')
    node.move(0.8)
    node.turn('right')
    node.move(1.1)
    node.turn('right')
    node.move(0.59)
    node.turn('left')
    node.move(0.45)
    

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



