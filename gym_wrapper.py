import gymnasium as gym
from gymnasium import spaces
import numpy as np
from simulation import Simulation

class TwoWheeledRobotEnv(gym.Env):
    def __init__(self, render=False):
        super(TwoWheeledRobotEnv, self).__init__()

        # Action space: velocities for left and right wheels
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # Observation space: robot position and orientation (x, y, theta, x_target, y_target)
        self.observation_space = spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.pi, -np.inf, -np.inf]),
            high=np.array([np.inf, np.inf, np.pi, np.inf, np.inf]),
            dtype=np.float32
        )

        self.simulation = Simulation(render=render)
        self.target_pos = np.array([2.0, 2.0])

        self.steps = 0

    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)

        # Reset the robot and randomize target position
        self.simulation.reset_robot()
        self.target_pos = np.random.uniform(-2, 2, size=2)

        self.simulation.render_balloon(self.target_pos)

        # Return initial observation
        obs = self._get_observation()
        # Reset step count
        self.steps = 0
        return obs, {}


    def step(self, action):
        # Scale action to control wheel velocities
        left_wheel_velocity, right_wheel_velocity = action

        self.simulation.set_wheel_velocity(left_wheel_velocity, right_wheel_velocity)
        self.simulation.step_simulation()

        # Get observation
        obs = self._get_observation()

        # Compute reward
        reward = self._compute_reward(obs)

        # Check if done
        terminated = self._is_done(obs)

        # Truncate the episode if it has been running for 1000 steps
        if self.steps > 4500:
            truncated = True
        else:
            truncated = False

        info = {"reward": reward}

        # Increment step count
        self.steps += 1

        return obs, reward, terminated, truncated, info

    def render(self, mode="human"):
        pass  # Rendering logic can be added here if needed

    def close(self):
        self.simulation.close()

    def _get_observation(self):
        robot_state = self.simulation.get_robot_state()
        balloon_pos = self.simulation.get_balloon_position()
        obs = np.concatenate((robot_state, balloon_pos))
        return obs

    def _compute_reward(self, obs):
        # Extract robot position and orientation
        robot_pos = obs[:2]
        theta = obs[2]

        # Compute direction to target
        direction_to_target = self.target_pos - robot_pos
        distance_to_target = np.linalg.norm(direction_to_target)
        direction_to_target /= (distance_to_target + 1e-6)  # Normalize to unit vector

        # Compute robot's heading vector
        robot_heading = np.array([np.cos(theta), np.sin(theta)])

        # Orientation alignment reward
        alignment = np.dot(robot_heading, direction_to_target)  # Dot product
        orientation_reward = alignment  # Higher when facing the target

        # Distance reward (encourage reducing distance)
        if hasattr(self, 'prev_distance_to_target'):
            distance_reward = self.prev_distance_to_target - distance_to_target
        else:
            distance_reward = 0

        self.prev_distance_to_target = distance_to_target

        # Combine rewards
        reward = 1.0 * orientation_reward + 1.0 * distance_reward

        # Penalize being far from the target
        reward -= 0.1 * distance_to_target

        return reward

    def _is_done(self, obs):
        # Episode ends if the robot is very close to the target
        robot_pos = obs[:2]
        distance_to_target = np.linalg.norm(robot_pos - self.target_pos)
        return distance_to_target < 0.3
