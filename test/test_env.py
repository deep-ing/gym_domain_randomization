from random import random
import gym
import gym_labyrinth
import time 

env = gym.make('Labyrinth-v1', connect_gui=False, random_agent_pos=0, physical_steps=10)
state = env.reset()

print(env.action_space)
print(env.observation_space)
timestep = 0
for j in range(100000):
    state, reward, done, info = env.step(env.action_space.sample())
    # time.sleep(0.001)
    if done:
        print(timestep)
        env.reset()
        timestep = 0
    timestep += 1

