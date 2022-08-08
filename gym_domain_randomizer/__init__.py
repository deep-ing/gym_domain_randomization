
import gym 

#TODO : rename the directory

gym.envs.register(
     id='Labyrinth-v0',
     entry_point='gym_domain_randomizer.Labyrinth:Labyrinth',
     max_episode_steps=100
     
)
