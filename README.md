## Quick Start

1.  **Build the Package:**
    Navigate to your ROS 2 workspace and build the `rl_robot` package.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select rl_robot
    ```

2.  **Source the Workspace:**
    Make the built package available in your current terminal session.
    ```bash
    source install/setup.bash
    ```
    *Note: You'll need to do this in every new terminal you open to use this package.*

3.  **Launch Gazebo + Robot:**
    In a terminal (where you've sourced the workspace), launch the Gazebo simulation environment with the robot model.
    ```bash
    ros2 launch rl_robot spawn_robot.launch.py
    ```
    *Wait for Gazebo to fully load.*

4.  **Start Training:**
    In a *separate* terminal (ensure you've sourced the workspace here too: `source ~/ros2_ws/install/setup.bash`), start the reinforcement learning training process.
    ```bash
    ros2 launch rl_robot training.launch.py
    ```
    *You should see training logs appear in this terminal.*