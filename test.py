import gym
import gym_labyrinth
import time 

env = gym.make('Labyrinth-v0')
env.connect_gui()
state = env.reset()

print(env.action_space)
print(env.observation_space)

for j in range(5000):
    state, reward, done, info = env.step(env.action_space.sample())
    time.sleep(0.000005)

