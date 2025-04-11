from stable_baselines3 import DQN
from rl_robot.gazebo_env import GazeboRobotEnv

env = GazeboRobotEnv()
model = DQN(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=1e-3,
    buffer_size=10000,
    batch_size=32,
    exploration_fraction=0.3,
    exploration_final_eps=0.05
)
model.learn(total_timesteps=5000)
model.save("dqn_robot_nav")
env.close()