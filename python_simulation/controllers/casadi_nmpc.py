from casadi import *
import random 
import matplotlib.pyplot as plt
import math
import time

class Casadi_nmpc:
    def __init__(self,horizon, input_constraint, state_constraints, dt):
        self.N = horizon
        self.input_constraint = input_constraint
        self.state_constraints = state_constraints

        self.n_actionsMPC = 2
        self.dt = dt
        self.numberState = 3

        self.Q = 100
        self.R = 0.1

        self.initialize_casadi()

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

            next_x = self.X_casadi[0,k] + self.U_casadi[0,k]*cos(self.X_casadi[2,k])*self.dt
            next_y = self.X_casadi[1,k] + self.U_casadi[0,k]*sin(self.X_casadi[2,k])*self.dt
            next_theta = self.X_casadi[2,k] + self.U_casadi[1,k]*self.dt

            self.opti.subject_to(self.X_casadi[0,k+1]==next_x) # close the gaps
            self.opti.subject_to(self.X_casadi[1,k+1]==next_y) # close the gaps
            self.opti.subject_to(self.X_casadi[2,k+1]==next_theta) # close the gaps   

        p_opts = dict(print_time=False, verbose=False) 
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt",p_opts,s_opts) # set numerical backend
        #self.opti.solver("sqpmethod") # set numerical backend  


    ################################################

    def compute_mpc(self, initial_state, reference_x, reference_y):
        #casadi constraints
        start_time = time.time()
        self.opti.subject_to(self.x_casadi[0]==initial_state[0])  
        self.opti.subject_to(self.y_casadi[0]==initial_state[1])   
        self.opti.subject_to(self.theta_casadi[0]==initial_state[2])

        position_error = 0
        input_use = 0
        #print("reference_x",reference_x)
        #print("reference_y",reference_y)
        #casadi cost function - tracking
        #self.opti.set_initial(self.x_casadi, reference_x)
        #self.opti.set_initial(self.y_casadi, reference_y)
        for k in range(1,self.N):
            ref_x = reference_x[k]
            ref_y = reference_y[k]
            position_error += self.Q*(self.x_casadi[k] - ref_x)@(self.x_casadi[k] - ref_x).T
            position_error += self.Q*(self.y_casadi[k] - ref_y)@(self.y_casadi[k] - ref_y).T
            #position_error += (0)*(self.theta_casadi[k] - 0.0)@(self.theta_casadi[k] - 0.0).T
      
        for k in range(0,self.N-1):
            ref_v = 0
            ref_w = 0
            input_use += self.R*(self.U_casadi[0,k] - ref_v)@(self.U_casadi[0,k] - ref_v).T 
            input_use += self.R*(self.U_casadi[1,k] - ref_w)@(self.U_casadi[1,k] - ref_w).T 

        
        self.opti.minimize(position_error + input_use)
        #print("initialization time", time.time()-start_time)
        start_time = time.time()
        sol = self.opti.solve()
        #print("solving time", time.time()-start_time)
        v = sol.value(self.U_casadi)[0]
        w = sol.value(self.U_casadi)[1]

        print("v",v[0])
        print("w",w[0])

        print("x", sol.value(self.x_casadi))
        print("y", sol.value(self.y_casadi))
        return v[0],w[0]