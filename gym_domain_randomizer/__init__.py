
import gym 

#TODO : rename the directory

gym.envs.register(
     id='Labyrinth-v0',
     entry_point='gym_domain_randomizer.Labyrinth:Labyrinth',
     max_episode_steps=100
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

gym.envs.register(
    id="CartPoleRandomizable-v1",
    entry_point="gym_domain_randomizer.CartPoleEnv:CartPoleEnv",
    max_episode_steps=500,
    reward_threshold=475.0,
)

gym.envs.register(
    id="AcrobotRandomizable-v1",
    entry_point='gym_domain_randomizer.AcrobotEnv:AcrobotEnv',
    reward_threshold=-100.0,
    max_episode_steps=500,
)