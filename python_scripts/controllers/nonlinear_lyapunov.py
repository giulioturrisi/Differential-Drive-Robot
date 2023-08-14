import random 
import math
import time
import numpy as np


class Nonlinear_lyapunov:
    """Class for the Nonlinear_lyapunov control law
    """
    def __init__(self, k1, k2, k3, dt):
        """Init func
        Args:
            k1 (float): gain number 1
            k2 (float): gain number 2
            k3 (float): gain number ...3!
            dt (float): sampling time
        """
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

        self.dt = dt


    def reset(self,):
        """Every control class should have a reset function
        """
        self.previous_reference = None
        self.previous_reference_dot = [0.0 ,0.0]


    def compute_control(self, initial_state, reference_x, reference_y):
        """Compute the control actions
        Args:
            initial_state (np.array): actual state of the robot
            reference_x (np.array): x reference for the robot
            reference_y (np.array): y reference for the robot
        Returns:
            (np.array): control actions
        """
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
        else:
            reference_x_dot = 0.01
            reference_y_dot = 0.01
            
            reference_x_ddot = 0.01
            reference_y_ddot = 0.01

        
        vd = math.sqrt(reference_x_dot*reference_x_dot + reference_y_dot*reference_y_dot)
        wd = (reference_y_ddot*reference_x_dot - reference_x_ddot*reference_y_dot)/(reference_x_dot*reference_x_dot + reference_y_dot*reference_y_dot)

        

        e1 = math.cos(state_yaw)*(reference_x - state_x) + math.sin(state_yaw)*(reference_y - state_y);
        e2 = -math.sin(state_yaw)*(reference_x - state_x) + math.cos(state_yaw)*(reference_y - state_y);

        reference_z = math.atan2(reference_x_dot, reference_y_dot)
        e3 = reference_z - state_yaw;


        u1 = self.k1*e1
        u2 = self.k2*np.sign(vd)*(math.sin(e3)/e3)*e2 + self.k3*e3
        #u2 = self.k2*vd*(math.sin(e3)/e3)*e2 + self.k3*e3

        v = vd*math.cos(e3) + u1;
        w = wd + u2;


        return v, w