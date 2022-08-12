from typing import Optional

import numpy as np
import gym


class StayEnv(gym.Env):
    def __init__(self):
        self.wind_r = 1.0
        self.wind_theta = 1.0

        self.friction_x = 1.0
        self.friction_y = 1.0

        self.action_space = gym.spaces.Box(-1, 1, shape=(2,))
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(2,))

        self.pos_agent = np.array([0.0, 0.0])
        self.pos_goal = np.array([0.0, 0.0])
        self.relative_pos = self.pos_goal - self.pos_agent

    def reset(self):
        super().reset()

        self.pos_agent = np.array([0.0, 0.0])
        self.pos_goal = np.array([0.0, 0.0])
        self.relative_pos = self.pos_goal - self.pos_agent

        return self.obs()

    def set_system_params(self, params):
        for k, v in params.items():
            setattr(self, k, v)

    def obs(self):
        return self.relative_pos
        #r = np.linalg.norm(self.relative_pos)
        #theta = np.arctan(self.relative_pos[1] / self.relative_pos[0])
        #if theta < 0:
        #    theta += 2*np.pi
        #return np.array([r, theta], dtype=np.float32)

    def step(self, action):
        self.pos_agent[0] += action[0]*self.friction_x
        self.pos_agent[1] += action[1]*self.friction_y
        #action_x = action[0]*np.cos(2*np.pi*action[1])
        #action_y = action[0]*np.sin(2*np.pi*action[1])
        #self.pos_agent[0] += action_x
        #self.pos_agent[1] += action_y

        wind_x = self.wind_r*np.cos(2*np.pi*self.wind_theta)
        wind_y = self.wind_r*np.sin(2*np.pi*self.wind_theta)
        # wind_x = self.wind_r
        # wind_y = self.wind_theta
        self.pos_agent[0] += wind_x
        self.pos_agent[1] += wind_y

        prev_distance = np.linalg.norm(self.relative_pos)
        self.relative_pos = self.pos_goal - self.pos_agent
        curr_distance = np.linalg.norm(self.relative_pos)

        progress_reward = prev_distance - curr_distance

        reach_reward = 0
        if curr_distance < 0.25:
            reach_reward += 1

        reward = progress_reward + reach_reward - 0.0

        return self.obs(), reward, False, {}


if __name__ == '__main__':
    env = StayEnv()
    a = env.reset()

    for _ in range(50):
        observation, reward, done, info = env.step(env.action_space.sample())
