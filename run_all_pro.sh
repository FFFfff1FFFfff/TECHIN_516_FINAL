#!/bin/bash

# Step 1 - Start FastDDS Discovery
gnome-terminal -- bash -c "fastdds discovery --server-id 0; exec bash"

sleep 5

# Step 2 - Build and Source 'my_new_trajectory'
gnome-terminal -- bash -c "
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select my_new_trajectory
source install/setup.bash
exec bash"

sleep 5

# Step 3 - Build and Source 'close_loop_control'
gnome-terminal -- bash -c "
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select close_loop_control
source install/setup.bash
exec bash"

sleep 5

# Terminal 1 - Launch gen3_lite
gnome-terminal -- bash -c "
source ~/ros2_ws/install/setup.bash
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=10.18.2.239 launch_rviz:=false
exec bash"

sleep 5

# Terminal 2 - Launch Kinova MoveIt Config
gnome-terminal -- bash -c "
source ~/ros2_ws/install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=10.18.2.239
exec bash"

sleep 5

# Terminal 3 - Run my_new_trajectory
gnome-terminal -- bash -c "
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select my_new_trajectory
source install/setup.bash
ros2 run my_new_trajectory run_real_final
exec bash"

sleep 150  # 3 minutes

# Terminal 4 - SSH and delay before launching TurtleBot3
# gnome-terminal -- bash -c "
# sshpass -p 'robot1234' ssh ubuntu@10.18.3.92'
# sleep 10;
# source ~/ros2_ws/install/setup.bash
# ros2 launch turtlebot3_bringup robot.launch.py'
# exec bash"

sleep 10

# Terminal 5 - Run close_loop_control
gnome-terminal -- bash -c "
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select close_loop_control
source install/setup.bash
ros2 run close_loop_control turtle_final_pro_back
exec bash"

echo "Succeed！"



