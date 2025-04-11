##Build the package:
cd ~/ros2_ws
colcon build --packages-select rl_robot
source install/setup.bash

##Launch Gazebo + Robot:
ros2 launch rl_robot spawn_robot.launch.py

##Start Training:
ros2 launch rl_robot training.launch.py
