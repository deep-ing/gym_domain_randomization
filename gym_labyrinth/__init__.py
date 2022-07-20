
import gym 

gym.envs.register(
     id='Labyrinth-v0',
     entry_point='gym_labyrinth.Labyrinth:Labyrinth',
)

gym.envs.register(
     id='Labyrinth-v1',
     entry_point='gym_labyrinth.Labyrinth:Labyrinth',
     max_episode_steps=1000
)