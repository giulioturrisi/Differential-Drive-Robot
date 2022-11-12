import random 
import math
import time
import numpy as np

import copy 

class Dynamic_linearization:
    def __init__(self, k1, k2, dt):
        self.k1 = k1
        self.k2 = k2


        self.dt = dt

        self.reset()

    def reset(self,):
        self.eps = 0.0

        self.previous_state = None     
        self.previous_reference = None
        self.previous_reference_dot = [0.0 ,0.0]



    def compute_control(self, initial_state, reference_x, reference_y):
        state_x = initial_state[0]
        state_y = initial_state[1]
        state_yaw = initial_state[2]


        if(self.previous_reference is not None):
            #reference vel
            reference_x_dot = (reference_x - self.previous_reference[0])/self.dt;
            reference_y_dot = (reference_y - self.previous_reference[1])/self.dt;

            #reference acc
            reference_x_ddot = (reference_x_dot - self.previous_reference_dot[0])/self.dt
            reference_y_ddot = (reference_y_dot - self.previous_reference_dot[1])/self.dt              

            #robot vel
            state_x_dot = (state_x - self.previous_state[0])/self.dt
            state_y_dot = (state_y - self.previous_state[1])/self.dt
            #state_yaw_dot = 0.0
        else:
            #reference vel
            reference_x_dot = 0.01 
            reference_y_dot = 0.01
            
            #reference acc
            reference_x_ddot = 0.01
            reference_y_ddot = 0.01

            
            #robot vel
            state_x_dot = 0.0
            state_y_dot = 0.0
            #state_yaw_dot = 0.0

            self.previous_state = copy.deepcopy(initial_state)



        
        '''if(reference_x_ddot > 0.3):
            reference_x_ddot = 0.3
        elif(reference_x_ddot < -0.3):
            reference_x_ddot = -0.3'''

        u1 = reference_x_ddot + self.k1*(reference_x_dot - state_x_dot) + self.k2*(reference_x - state_x)
        u2 = reference_y_ddot + self.k1*(reference_y_dot - state_y_dot) + self.k2*(reference_y - state_y)


        self.eps = self.eps + (u1*math.cos(state_yaw) + u2*math.sin(state_yaw))*self.dt
        if(math.fabs(self.eps) < 0.001):
            if(self.eps > 0):
                self.eps = 0.001
            else:
                self.eps = -0.001
            

        v = self.eps
        w = (-u1*math.sin(state_yaw) + u2*math.cos(state_yaw))/v

        #previous state
        self.previous_state[0] = state_x
        self.previous_state[1] = state_y
        
        #previous reference
        self.previous_reference = [reference_x, reference_y] 
        self.previous_reference_dot = [reference_x_dot, reference_y_dot]

        return v, w