import gym
import gym_labyrinth
import time 
import numpy as np 
import matplotlib
import matplotlib.pyplot as plt

import os 

if not os.path.exists("results"):
    os.mkdir("results")

env = gym.make('Labyrinth-v0', connect_gui=False, random_agent_pos=0)
obs = env.reset()

for i in range(2000):
    if i%500 ==0:
        plt.imshow(obs)
        plt.savefig(f"results/test_{i}.png")
    obs, _,_,_ = env.step(env.action_space.sample())
print(obs.shape)