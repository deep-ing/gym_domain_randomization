from os import system
import gym 
import pybullet as p
from gym_labyrinth.maps import GridMap1
from gym_labyrinth.PhysicalEnv import PhysicalEnv
import numpy as np 
import pybullet_data 


AGENT_INFO = {
        "globalScaling" : 5,
        "acc" : 4.3,
        "max_speed" : 10,
        "color" : [0,125,0,1]
    }

OBSTACLE_INFO = {"globalScaling" : 20,
        "color" : [0,0,0,0.5],
        "acc" : 0.0001,
        "max_speed" : 2}

MAP_SIZE = 10 

class Labyrinth(PhysicalEnv):
    def __init__(self, connect_gui=False, random_agent_pos=7):
        
        super().__init__(MAP_SIZE, None, AGENT_INFO, OBSTACLE_INFO)
        
        # wind (2 theta, magnitude)  # friction (4 direction)
        self.set_system_params(np.zeros(6,))
        self.map = GridMap1()
        self.num_obstacles = self.map.num_obstacles
        
        self.action_space = gym.spaces.Discrete(6) 
        
        # TODO : define the observation space
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(10,)) 
        self.connect(connect_gui)
        self.random_agent_pos = random_agent_pos
        
        
    def connect(self, connect_gui):
        if connect_gui:
            p.connect(p.GUI)            
        else:
            p.connect(p.DIRECT)    
            
    
    def get_random_system_params(self):
        system_vector = np.random.random(6,)
        return system_vector
    
    def set_system_params(self, system_vector):
        assert system_vector.shape == (6,)
        self.system_vector = system_vector
    
    def apply_system_params(self, agent):
        # wind effect 
        r, theta = self.system_vector[:2]
        x, y = (r*np.cos(2*np.pi*theta), r*np.sin(2*np.pi*theta))
        force = [x,y,0]
        p.applyExternalForce(agent.pid, -1, 
                            forceObj=force,
                            posObj=agent.position,
                            flags=p.WORLD_FRAME) 
        # directional friction 
        agent.direction_friction = self.system_vector[2:]
        # ======================
        
    def reset(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        if self.objects:
            for object_type, object_list in self.objects.items():
                for obj in object_list:
                    obj.remove()
                object_list.clear()
        for _ in range(0): # no target
            self.build_position("target", [t-i for t,i in zip(self.map.target_position, self.map.init_position)] , **self.target_info)
        for _ in range(1): # only single agent
            # Randomly starts at the left line
            self.build_position("agent",  [a-i for a,i in zip(self.map.agent_position + np.array([0,-self.random_agent_pos*np.random.random(),0 ]), self.map.init_position)], **self.agent_info)
        for r in range(self.map.width):
            for c in range(self.map.height):
                if self.map.map1[r][c] == 1:
                    self.build_position("obstacle", [r-self.map.init_position[0], c-self.map.init_position[1] ,0], **self.obstacle_info)
        for obj in self.objects["obstacle"]:
            p.changeDynamics(obj.pid, -1, mass=100000)

    def step(self, agent_action):
        agent= self.objects['agent'][0]
        self.apply_system_params(agent)
        agent.take_action(agent_action)     
        # Apply system parameters        
        p.stepSimulation()
        for object_type, object_list in self.objects.items():
            for obj in object_list:
                if obj.alive:
                    obj.update()
                    obj.decrease_velocity()
                    obj.clip_velocity()

        state =np.hstack([self.objects['agent'][0].position,
                                   self.objects['agent'][0].velocity])

        reward = self._reward(agent)
        done = self._done(agent)
        info = self._info()
        return state, reward, done, info

    def _reward(self, agent): 
        reward = agent.position[1]  # y value itself 
        return reward 

    def _done(self, agent):
        done = False
        
        # 3.0 is almost 
        if agent.position[0] > 3.0:
            done = True 

        return done
         
    def _info(self):
        return {}

import time 
if __name__ == "__main__":

    system_vector = np.random.random(size=(6,))
    env = Labyrinth(True, 10, system_vector)
    env.reset()        
    
    for j in range(5000):
        state, reward, done, info = env.step(env.action_space.sample())
        if done:
            env.reset()
        time.sleep(0.000005)
