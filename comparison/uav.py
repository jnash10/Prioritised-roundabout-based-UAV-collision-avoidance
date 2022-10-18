from variables import *

class UAV:
    def __init__(self, position, goal, priority, velocity):
        self.p = position
        self.g = goal
        self.priority = priority
        self.v=velocity
        self.drones = []
        self.vmax = velocity
        self.kv = (self.vmax/rhonaught)**2



