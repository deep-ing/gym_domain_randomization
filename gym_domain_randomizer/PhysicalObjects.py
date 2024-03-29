import pybullet as p 
import numpy as np 

class PhysicalObjects:
    def __init__(self, initial_position, **kwargs):
        self.position = initial_position
        self.velocity = [0,0,0]
        self.max_speed = kwargs.get("max_speed", 2)   
        self.acc = kwargs.get("acc", 0.1)
        self.speed_decreasing = 0.999
        self.globalScaling = kwargs.get("globalScaling", 1)
        self.pid = p.loadURDF(kwargs.get("urdf", "cube_small.urdf"), initial_position, globalScaling=self.globalScaling)
        self.safe_boundary = kwargs.get("safe_boundary", 0.1)
        self.move_kind = "random_direction"
        self.alive = True
        p.changeDynamics(self.pid, -1, mass=kwargs.get("mass", 0.01))
        if "color" in kwargs:
            p.changeVisualShape(self.pid, -1, rgbaColor=kwargs.get("color", [0,0,125,1]))

    def move(self, kind, bound, **kwargs):
        if np.linalg.norm(self.position) > bound:
            force = [-self.position[0]/bound, -self.position[1]/bound, -self.position[2]/bound]
            p.applyExternalForce(self.pid, -1, 
                            forceObj=force,
                            posObj=self.position,
                            flags=p.WORLD_FRAME)
            return 

        force = [0, 0, 0]
        if kind=="circle_motion":
            period = kwargs.get("period")
            theta = kwargs.get("theta")
            radius = kwargs.get("radius")
            p.resetBaseVelocity(self.pid, [0,0,0])
            next_pos = [radius * np.cos(theta + np.pi / period *2), 
                        radius * np.sin(theta + np.pi / period *2)]
            force = [next_pos[i] - self.position[i] for i in range(2)] +[0]
        
        elif kind=="random_direction":
            randomness1 = np.random.uniform(-1,1) 
            randomness2 = np.random.uniform(-1,1) 
            force = [randomness1*self.acc , randomness2*self.acc , 0]
        elif kind=="x+":
            force = [0,0,0]
            p.resetBaseVelocity(self.pid,  [-self.acc*10, 0, 0])
        elif kind=="x-":
            force = [0,0,0]
            p.resetBaseVelocity(self.pid,  [self.acc*10, 0, 0])
        elif kind=="y+":
            force = [0,0,0]
            p.resetBaseVelocity(self.pid, [0, self.acc*10, 0])
        elif kind=="y-":
            force = [0,0,0]
            p.resetBaseVelocity(self.pid, [0, -self.acc*10, 0])
        elif kind=="with_velocity":
            p.resetBaseVelocity(self.pid, kwargs['velocity'])
        else:
            raise ValueError("Undefined Movement...")        
        
        self.move_kind = kind
        p.applyExternalForce(self.pid, -1, 
                            forceObj=force,
                            posObj=self.position,
                            flags=p.WORLD_FRAME)

    def clip_velocity(self):
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            velocity = [v / speed * self.max_speed for v in self.velocity]
            p.resetBaseVelocity(self.pid, velocity)

    def decrease_velocity(self):
        velocity = [v * self.speed_decreasing for v in self.velocity]
        p.resetBaseVelocity(self.pid, velocity)
        
    def update(self):
        self.position = p.getBasePositionAndOrientation(self.pid)[0]
        self.velocity = p.getBaseVelocity(self.pid)[0]
        
    def remove(self):
        if self.alive:
            self.alive=False
            p.removeBody(self.pid)


from gym.spaces import Discrete, Box, Dict
class Agent(PhysicalObjects):
    dx = [0, 0, -1,1,0] # right, left, up (reversed), down 
    dy = [1,-1, 0, 0,0]
    def __init__(self, initial_position, continuous, **kwargs):
        super().__init__(initial_position, **kwargs)
        self.action_size = kwargs.get('action_size', 6)
        self.action_space = Discrete(self.action_size)
        self.observation_space = Dict({
                                        'own_position' :Box(low=-np.inf, high=np.inf, shape=(3,)), 
                                        'own_velocity': Box(low=-np.inf, high=np.inf, shape=(3,))
                                    }) 
        self.direction_friction = np.zeros(shape=(4,))
        self.continuous = continuous


    def take_action(self, action):
        if self.continuous:
            # up down left right

            force = [-action[2] * (1-self.direction_friction[2]) + action[3] * (1-self.direction_friction[3]) , 
                     -action[0] * (1-self.direction_friction[0]) + action[1] * (1-self.direction_friction[1]) , 
                     0]
            p.applyExternalForce(self.pid, -1, 
                                        forceObj = force,
                                        posObj=self.position,
                                        flags=p.WORLD_FRAME) 
        else:
            if action < 8 :
                action = action%4
                acc = self.acc //2 if  (action//4) == 1 else self.acc
                friction = (1-self.direction_friction[action])
                force = [Agent.dx[action]*acc*friction, 
                         Agent.dy[action]*acc*friction, 
                         0]
                p.applyExternalForce(self.pid, -1, 
                                        forceObj = force,
                                        posObj=self.position,
                                        flags=p.WORLD_FRAME) 
            elif action == 8 :
                p.resetBaseVelocity(self.pid, [0,0,0])
            else:
                raise ValueError("Undefined action %d" %action)

    def relative_position(self, other):
        reltaive = np.array([other.position[i] - self.position[i] for i in range(3)])
        return reltaive

    def relative_velocity(self, other):
        relative = np.array([other.velocity[i] - self.velocity[i] for i in range(3)])
        return relative

    def distance(self, other, measure="euclidian"):
        if measure == "euclidian":
            return np.linalg.norm(self.relative_position(other))
        elif measure == "manhattan":
            return np.linalg.norm(self.relative_position(other), ord=np.inf)
        else:
            raise ValueError()


