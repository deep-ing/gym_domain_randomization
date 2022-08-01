

import gym 
import mujoco_py 
from gym.envs.mujoco import mujoco_env
from gym.envs.mujoco.half_cheetah import HalfCheetahEnv
import xml.etree.ElementTree as et
import os 
from gym import utils
import numpy as np 
from gym.spaces import Box
from gym import utils

class Env(HalfCheetahEnv):
    def __init__(self, **kwargs):
        observation_space = Box(low=-np.inf, high=np.inf, shape=(17,), dtype=np.float64)
        mujoco_env.MujocoEnv.__init__(self, kwargs.get('xml_name'), frame_skip=5)
        utils.EzPickle.__init__(self)
        xml_list = os.listdir(os.path.join(os.path.dirname(mujoco_env.__file__), "assets"))


        # randomization
        self.reference_path = os.path.join(os.path.dirname(mujoco_env.__file__), "assets", kwargs.get('xml_name'))
        self.reference_xml = et.parse(self.reference_path)

        self.root = self.reference_xml.getroot()
        self.xml = self.root
        self.system_vector = np.zeros(6)


    def set_system_params(self, vector):
        self.system_vector = vector
        #TODO define the vector change

        # for i, bodypart in enumerate(self.dimensions):
        #     for geom in self.dimension_map[i]:
        #         suffix = self.suffixes[i]
        #         value = "{:3f} {}".format(self.dimensions[i].current_value, suffix)
        #         geom.set('size', '{}'.format(value))
        xml = et.tostring(self.root, encoding='unicode', method='xml')
        self.xml = xml
        self._re_init(xml)
        return 

    def _re_init(self, xml):
        self.model = mujoco_py.load_model_from_xml(xml)
        self.sim = mujoco_py.MjSim(self.model)
        self.data = self.sim.data
        self.init_qpos = self.data.qpos.ravel().copy()
        self.init_qvel = self.data.qvel.ravel().copy()
        observation, _reward, done, _info = self.step(np.zeros(self.model.nu))



env = Env(xml_name="half_cheetah.xml")
xml = env.set_system_params(np.zeros(6,))

done = False 
for i in range(100):
    env.set_system_params(np.random.random(6,))
    env.reset()
    for j in range(10):
        print(j)
        env.step(env.action_space.sample())
