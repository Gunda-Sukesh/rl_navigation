import gym
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class GazeboRobotEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # Initialize ROS 2
        rclpy.init()
        self.node = Node('rl_robot_env')

        # Subscribers
        self.scan_sub = self.node.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.latest_scan = np.zeros(10)
        self.latest_imu = np.zeros(3)  # Linear acceleration (x,y,z)
        self.current_position = np.array([0.0, 0.0])
        self.goal_position = np.array([5.0, 0.0])  # Target at x=5, y=0

        # Action space (Discrete: Forward, Left, Right)
        self.action_space = gym.spaces.Discrete(3)

        # Observation space (LiDAR + IMU + Relative Goal)
        self.observation_space = gym.spaces.Box(
            low=np.array([0.0] * 10 + [-10.0] * 3 + [-10.0, -10.0]),
            high=np.array([10.0] * 10 + [10.0] * 3 + [10.0, 10.0]),
            dtype=np.float32
        )

    def scan_callback(self, msg):
        self.latest_scan = np.array(msg.ranges[::36])  # Downsample to 10 values

    def imu_callback(self, msg):
        self.latest_imu = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def odom_callback(self, msg):
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def step(self, action):
        # Execute action
        cmd = Twist()
        if action == 0:  # Forward
            cmd.linear.x = 0.2
        elif action == 1:  # Left
            cmd.angular.z = 0.5
        elif action == 2:  # Right
            cmd.angular.z = -0.5
        self.cmd_vel_pub.publish(cmd)

        # Wait for new state
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Calculate reward
        distance_to_goal = np.linalg.norm(self.current_position - self.goal_position)
        reward = -distance_to_goal * 0.1  # Penalize distance

        # Check termination
        done = any(self.latest_scan < 0.3) or distance_to_goal < 0.5

        # Observation
        observation = np.concatenate([
            self.latest_scan,
            self.latest_imu,
            self.goal_position - self.current_position
        ])

        return observation, reward, done, {}

    def reset(self):
        # Reset robot (simplified)
        self.current_position = np.array([0.0, 0.0])
        return np.concatenate([np.zeros(10), np.zeros(3), self.goal_position])

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()