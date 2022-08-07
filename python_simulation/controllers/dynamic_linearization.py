import random 
import math
import time
import numpy as np


class Dynamic_linearization:
    def __init__(self, k1, k2, dt):
        self.k1 = k1
        self.k2 = k2
     
        self.previous_reference = None
        self.previous_reference_dot = [0.0 ,0.0]

        self.dt = dt
        self.previous_state = None


    def compute_control(self, initial_state, reference_x, reference_y):
        state_x = initial_state[0]
        state_y = initial_state[1]
        state_yaw = initial_state[2]


        if(self.previous_reference is not None):
            previous_reference_x = self.previous_reference[0]
            previous_reference_y = self.previous_reference[1] 

            reference_x_dot = (reference_x - previous_reference_x)/self.dt;
            reference_y_dot = (reference_y - previous_reference_y)/self.dt;

            reference_x_ddot = (reference_x_dot - self.previous_reference_dot[0])/self.dt
            reference_y_ddot = (reference_y_dot - self.previous_reference_dot[1])/self.dt              
            
            self.previous_reference = [reference_x, reference_y] 
            self.previous_reference_dot = [reference_x_dot, reference_y_dot]

            state_x_dot = (state_x - self.previous_state[0])/self.dt
            state_y_dot = (state_y - self.previous_state[1])/self.dt
            state_yaw_dot = 0.0
        else:
            reference_x_dot = 0.01
            reference_y_dot = 0.01
            
            reference_x_ddot = 0.01
            reference_y_ddot = 0.01
            
            self.previous_state = initial_state
            state_x_dot = 0.0
            state_y_dot = 0.0
            state_yaw_dot = 0.0

            self.eps = reference_x_dot*math.cos(state_yaw) + reference_y_dot*math.sin(state_yaw) 

          
        

        u1 = reference_x_ddot + self.k1*(reference_x_dot - state_x_dot) + self.k2*(reference_x - state_x)
        u2 = reference_y_ddot + self.k1*(reference_y_dot - state_y_dot) + self.k2*(reference_y - state_y)


        self.eps = self.eps + (u1*math.cos(state_yaw) + u2*math.sin(state_yaw))*self.dt

        v = self.eps
        w = u2*math.cos(state_yaw) - u1*math.sin(state_yaw)/self.eps


        if(self.eps < 0.001):
            self.eps = reference_x_dot*math.cos(state_yaw) + reference_y_dot*math.sin(state_yaw) 

        return v, w