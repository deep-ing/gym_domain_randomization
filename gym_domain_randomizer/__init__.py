
import gym 

#TODO : rename the directory

gym.envs.register(
     id='Labyrinth-v0',
     entry_point='gym_labyrinth.Labyrinth:Labyrinth',
     max_episode_steps=500
     
)
