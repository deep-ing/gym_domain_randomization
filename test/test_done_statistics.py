from random import random
import gym
import gym_labyrinth
import time 
from tqdm import tqdm 

env = gym.make('Labyrinth-v0', connect_gui=False)
state = env.reset()

print(env.action_space)
print(env.observation_space)
episodes = 100
timestep = [0 for i in range(episodes)]

for i in tqdm(range(episodes)):
    done = False
    env.reset()
    while not done:
        state, reward, done, info = env.step(env.action_space.sample())
        timestep[i] += 1
    
import numpy as np
print("Force done at timestep == 500")
print("Num Episodes:", len(timestep))
print("Timestep Quantiles:" ,[np.quantile(timestep, i/9) for i in range(10)])
print("Timestep Mean:" ,np.mean(timestep))
print("Timestep STD :" ,np.std(timestep))
