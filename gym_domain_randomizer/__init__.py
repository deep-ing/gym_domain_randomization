
import gym 

#TODO : rename the directory

gym.envs.register(
     id='Labyrinth-v0',
     entry_point='gym_domain_randomizer.Labyrinth:Labyrinth',
     max_episode_steps=500    
)

gym.envs.register(
     id='StayRandomizable-v0',
     entry_point='gym_domain_randomizer.StayEnv:StayEnv',
     max_episode_steps=100
)

gym.envs.register(
     id='PendulumRandomizable-v1',
     entry_point='gym_domain_randomizer.PendulumEnv:PendulumEnv',
     max_episode_steps=200
)