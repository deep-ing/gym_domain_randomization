from random import random
import gym
import gym_labyrinth
import time 
import numpy as np

env = gym.make('Labyrinth-v0', connect_gui=True)
state = env.reset()
total_reward=0
print(env.action_space)
print(env.observation_space)

for j in range(100000):

    state, reward, done, info = env.step(env.action_space.sample())
    total_reward += reward 
    # print(env.objects['agent'][0].position)
    if j%5000 ==0:
        system_vector = np.zeros(6,)
        env.set_system_params(system_vector)
        done=True
    if done:
        print(total_reward)
        total_reward = 0
        env.reset()
        
    
        

