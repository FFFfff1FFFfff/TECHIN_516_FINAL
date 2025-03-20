import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import GripperInterface, MoveIt2, MoveIt2State
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import csv
import time

class KinovaPickPlace(Node):
    def __init__(self):
        super().__init__('kinova_pick_place')
        self.callback_group = ReentrantCallbackGroup()

        # Initialize MoveIt2 interface for motion control
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
            base_link_name='base_link',
            end_effector_name='end_effector_link',
            group_name='arm',
            callback_group=self.callback_group,
        )

        # Initialize Gripper Interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=['right_finger_bottom_joint'],
            open_gripper_joint_positions=[0.85],
            closed_gripper_joint_positions=[0.0],
            gripper_group_name='gripper',
            callback_group=self.callback_group,
            gripper_command_action_name='gripper_action_controller/gripper_cmd',
        )

        # Set planning parameters
        self.moveit2.planner_id = 'RRTConnectkConfigDefault'
        self.moveit2.allowed_planning_time = 5.0
        self.moveit2.num_planning_attempts = 10
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

    def control_gripper(self, gripper_value):
        """
        控制夹爪的开闭
        参数：
        - gripper_value: 0.0 (完全打开) ~ 0.85 (完全闭合)
        """
        self.get_logger().info(f"Moving gripper to position {gripper_value}")
        self.gripper.move_to_position(gripper_value)
        self.gripper.wait_until_executed()
        time.sleep(1)

    def move_to_pose(self, pose: Pose):
        """移动到指定姿态"""
        self.get_logger().info(f"Moving to position: {pose.position.x}, {pose.position.y}, {pose.position.z}")
        self.moveit2.move_to_pose(pose)

        rate = self.create_rate(10)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()
        future = self.moveit2.get_execution_future()
        while not future.done():
            rate.sleep()

    def execute_task(self):
        # 读取 CSV 文件
        points = []
        with open('/home/yifanli8@netid.washington.edu/ros2_ws/src/points.csv', 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                pos_x = float(row['pos_x'])
                pos_y = float(row['pos_y'])
                pos_z = float(row['pos_z'])
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                w = float(row['w'])
                points.append((pos_x, pos_y, pos_z, x, y, z, w))

        # 执行动作序列
        for i, point in enumerate(points):
            pos_x, pos_y, pos_z, x, y, z, w = point

            target_pose = Pose()
            target_pose.position = Point(x=pos_x, y=pos_y, z=pos_z)
            target_pose.orientation = Quaternion(x=x, y=y, z=z, w=w)

            self.get_logger().info(f"Moving to point {i+1}")
            self.move_to_pose(target_pose)

            # 根据动作的不同进行夹爪控制
            if i == 1:  # 抓取位置时，夹爪半闭合
                self.get_logger().info("Closing gripper at grab position")
                self.control_gripper(0.2)
            elif i == 2:  # 抓起位置时，夹爪完全闭合
                self.get_logger().info("Gripper fully closed at pick position")
                self.control_gripper(0.0)
            elif i == 4:  # 放置位置时，夹爪打开
                self.get_logger().info("Releasing object at place position")
                self.control_gripper(0.85)

def main():
    rclpy.init()
    node = KinovaPickPlace()
    node.execute_task()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




