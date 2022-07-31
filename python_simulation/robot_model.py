import numpy as np
import math 

class Robot:
    def __init__(self, dt):
        #self.radius = radius
        #self.diameter = diameter
        self.dt = dt

    
    def integrate(self, state, v, w):
        state[0] = state[0] + v*math.cos(state[2])*self.dt
        state[1] = state[1] + v*math.sin(state[2])*self.dt
        state[2] = state[2] + w*self.dt
        return state