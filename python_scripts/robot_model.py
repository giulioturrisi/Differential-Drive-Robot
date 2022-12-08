import numpy as np
import math 
import casadi as cs


class Robot:
    def __init__(self, dt):
        self.dt = dt

        state = cs.SX.sym("state", 3, 1)
        control = cs.SX.sym("control", 2, 1)
 
        self.A_f = self.get_A_f_matrix(state, control)
        self.B_f = self.get_B_f_matrix(state, control)

    
    def integrate(self, state, v, w):
        # Simple unycicle kynematics ---------------------------------------
        state[0] = state[0] + v*cs.np.cos(state[2])*self.dt
        state[1] = state[1] + v*cs.np.sin(state[2])*self.dt
        state[2] = state[2] + w*self.dt

        print("v",v)
        print("state",state)
        return state

    def fk(self, state, u):
        # Simple unycicle kynematics ---------------------------------------
        x_d = u[0]*cs.np.cos(state[2])
        y_d = u[0]*cs.np.sin(state[2])
        yaw_d = u[1]
        return np.array([x_d, y_d, yaw_d])

    def get_A_f_matrix(self,state, control):
        state_sym = cs.SX.sym("state", 3, 1)
        control_sym = cs.SX.sym("control", 2, 1)
        forward_kinematics_f = self.fk(state_sym, control_sym)
        

        A = cs.jacobian(forward_kinematics_f, state_sym)
        A_f = cs.Function("A", [state_sym, control_sym], [A])

        return A_f

    def get_B_f_matrix(self, state, control):
        state_sym = cs.SX.sym("state", 3, 1)
        control_sym = cs.SX.sym("control", 2, 1)
        forward_kinematics_f = self.fk(state_sym, control_sym)
   
        B = cs.jacobian(forward_kinematics_f, control_sym)
        B_f = cs.Function("B", [state_sym, control_sym], [B])

        print("B_f", B_f)

        return B_f


