import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv


def load_poses_from_csv(file_path):
    with open(file_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        poses = {
            row['name']: Pose(
                position=Point(
                    x=float(row['pos_x']),
                    y=float(row['pos_y']),
                    z=float(row['pos_z'])
                ),
                orientation=Quaternion(
                    x=float(row['x']),
                    y=float(row['y']),
                    z=float(row['z']),
                    w=float(row['w'])
                )
            )
            for row in reader
        }
    return poses


def execute_movements(arm, gripper, movements):
    for action in movements:
        if isinstance(action, Pose):
            arm.inverse_kinematic_movement(action)
        elif isinstance(action, float):
            gripper.move_to_position(action)
        time.sleep(1)


def main():
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    # arm.go_vertical()
    gripper.move_to_position(0.1)

    poses = load_poses_from_csv("/home/yifanli8@netid.washington.edu/ros2_ws/src/my_new_trajectory/my_new_trajectory/eight_2.csv")

    movements = [
        poses['up'],
        poses['grasp'],
        0.6,
        poses['mid_1'],
        poses['mid_2'],
        poses['drop'],
        0.02
    ]

    execute_movements(arm, gripper, movements)

    gripper.shutdown()
    arm.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
