from casadi import *
import random 
import matplotlib.pyplot as plt
import math

class nmpc:
    def __init__(self,horizon, input_constraint, state_constraints):
        self.N = horizon
        self.input_constraint = input_constraint
        self.state_constraints = state_constraints

        self.n_actionsMPC = 2
        self.dt = 0.01
        self.numberState = 3

        self.Q = 100
        self.R = 1

    #INIT CASADI PLANNING
    def initialize_casadi(self):
        self.opti = Opti() # Optimization problem
        # ---- decision variables ---------
        self.X_casadi = self.opti.variable(self.numberState,self.N+1) # state trajectory
        self.x_casadi   = self.X_casadi[0,:]
        self.y_casadi   = self.X_casadi[1,:]
        self.theta_casadi   = self.X_casadi[2,:]
            
        self.U_casadi = self.opti.variable(self.n_actionsMPC,self.N)   # control trajectory


        # ---- dynamic constraints --------
        f = lambda x,u: vertcat(x[1],u-x[1]) # dx/dt = f(x,u)
        for k in range(self.N): # loop over control intervals

            next_x = self.X_casadi[0,k] + self.U_casadi[0,k]*math.cos(self.X_casadi[2,k])*self.dt
            next_y = self.X_casadi[1,k] + self.U_casadi[0,k]*math.sin(self.X_casadi[2,k])*self.dt
            next_theta = self.X_casadi[2,k] + self.U_casadi[1,k]*self.dt

            self.opti.subject_to(self.X_casadi[0,k+1]==next_x) # close the gaps
            self.opti.subject_to(self.X_casadi[1,k+1]==next_y) # close the gaps
            self.opti.subject_to(self.X_casadi[2,k+1]==next_theta) # close the gaps   

        p_opts = dict(print_time=False, verbose=False) 
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt",p_opts,s_opts) # set numerical backend
        #self.opti.solver("sqpmethod") # set numerical backend  


    ################################################

    def compute_mpc(self, reference, initial_state):
        #casadi constraints
        self.opti.subject_to(self.x_casadi[0]==self.x[0])  
        self.opti.subject_to(self.y_casadi[0]==self.x[1])   
        self.opti.subject_to(self.theta_casadi[0]==self.x[2])

        position_error = 0
        input_use = 0
        #casadi cost function - tracking
        for k in range(1,self.N):
            if(self.t + k < self.N):
                ref_x = self.reference_x[self.t + k]
                ref_y = self.reference_y[self.t + k]
            else:
                ref_x = self.reference_x[self.N]
                ref_y = self.reference_y[self.N]
        position_error += self.Q*(self.x_casadi_anc[k] - ref_x)@(self.x_casadi_anc[k] - ref_x).T
        position_error += self.Q*(self.y_casadi_anc[k] - ref_y)@(self.y_casadi_anc[k] - ref_y).T
      
        for k in range(0,self.N-1):
            if(self.t + k < self.N - 1):
                #ref_u1 = self.u_1[self.t + k]
                #ref_u2 = self.u_2[self.t + k]
                ref_v = 0
                ref_w = 0
            else:
                ref_v = 0
                ref_w = 0
        input_use += self.R*(self.U_casadi_anc[0,k] - ref_v)@(self.U_casadi_anc[0,k] - ref_v).T 
        input_use += self.R*(self.U_casadi_anc[1,k] - ref_w)@(self.U_casadi_anc[1,k] - ref_w).T 
        

        self.opti.minimize(position_error + input_use)

        sol = self.opti.solve()
        self.v = sol.value(self.U_casadi)[0]
        self.w = sol.value(self.U_casadi)[1]
        return [v,w]