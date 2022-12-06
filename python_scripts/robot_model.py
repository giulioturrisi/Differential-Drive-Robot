import numpy as np
import math 
import casadi as cs


class Robot:
    def __init__(self, dt):
        self.dt = dt

    
    def integrate(self, state, v, w):
        # Simple unycicle kynematics ---------------------------------------
        state[0] = state[0] + v*cs.np.cos(state[2])*self.dt
        state[1] = state[1] + v*cs.np.sin(state[2])*self.dt
        state[2] = state[2] + w*self.dt
        return state

    def fk(self, state, u):
        # Simple unycicle kynematics ---------------------------------------
        x_d = u[0]*cs.np.cos(state[2])
        y_d = u[0]*cs.np.sin(state[2])
        yaw_d = u[1]
        return np.array([x_d, y_d, yaw_d])