from random import random
import gym
import gym_labyrinth
import time 

env = gym.make('Labyrinth-v1', connect_gui=True, random_agent_pos=0)
state = env.reset()

print(env.action_space)
print(env.observation_space)

for j in range(100000):
    if j%1000 ==0:
        system_vector = env.get_random_system_params()
        env.set_system_params(system_vector)
        print(env.system_vector)
        env.reset()
    state, reward, done, info = env.step(env.action_space.sample())
    time.sleep(0.001)
    if done:
        env.reset()

