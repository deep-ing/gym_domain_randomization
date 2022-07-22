from random import random
import gym
import gym_labyrinth
import time 

env = gym.make('Labyrinth-v0', connect_gui=True)
state = env.reset()

print(env.action_space)
print(env.observation_space)
timestep = 0
for j in range(100000):
    state, reward, done, info = env.step(env.action_space.sample())
    # time.sleep(0.001)
    if done:
        print(timestep)
        sv = env.get_random_system_params()
        env.set_system_params(sv)
        env.reset()
        timestep = 0
    timestep += 1

