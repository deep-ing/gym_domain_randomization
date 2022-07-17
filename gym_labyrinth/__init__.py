
import gym 

gym.envs.register(
     id='Labyrinth-v0',
     entry_point='gym_labyrinth/labyrinth:Labyrinth',
     max_episode_steps=1000,
)