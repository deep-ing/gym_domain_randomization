import gym 
import gym_labyrinth
import numpy as np

def make_env(env_id, seed, idx):
    def thunk():
        env = gym.make(env_id, system_random_interval=[[(0,0.1)], # list of the first  interval
                                                        [(0,0.2), (0.4,0.5)], # list of the second interval
                                                        [(0.99,1)],
                                                        [(0.01,0.03), (0.09, 0.1)],
                                                        [(0,1)],
                                                        [(0,1)]])
        env = gym.wrappers.RecordEpisodeStatistics(env)
        env.seed(seed)
        env.action_space.seed(seed)
        env.observation_space.seed(seed)
        return env
    return thunk

num_envs = 4
envs_list = [make_env("Labyrinth-v1", 0, i) for i in range(num_envs)]
envs = gym.vector.SyncVectorEnv(
    envs_list
)

states = envs.reset()
for i in range(10000):
    actions = np.array([envs.single_action_space.sample() for i in range(num_envs)])
    next_obs, reward, done, info = envs.step(actions)
    if True in done:
        for i in range(num_envs):
            if done[i]:
                print(info[i].keys())
                print(info[i]['system_vector'])
                system_vector = envs.envs[i].get_random_system_params()
                envs.envs[i].set_system_params(system_vector)