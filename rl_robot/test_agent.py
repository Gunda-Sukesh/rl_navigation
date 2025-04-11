from stable_baselines3 import DQN
from rl_robot.gazebo_env import GazeboRobotEnv

env = GazeboRobotEnv()
model = DQN.load("dqn_robot_nav")

obs = env.reset()
while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, _, done, _ = env.step(action)
    if done:
        obs = env.reset()