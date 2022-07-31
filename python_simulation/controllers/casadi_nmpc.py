from casadi import * # type: ignore
import random 
import matplotlib.pyplot as plt # type: ignore
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
        # Casadi problem formulation ---------------------------------------
        self.opti = Opti() # Optimization problem
        
        # Decision variables ---------------------------------------
        self.X_casadi = self.opti.variable(self.numberState,self.N+1) # state trajectory
        self.x_casadi   = self.X_casadi[0,:]
        self.y_casadi   = self.X_casadi[1,:]
        self.theta_casadi   = self.X_casadi[2,:]
            
        self.U_casadi = self.opti.variable(self.n_actionsMPC,self.N)   # control trajectory


        # Kynematic Constraints ---------------------------------------
        f = lambda x,u: vertcat(x[1],u-x[1]) # dx/dt = f(x,u)
        for k in range(self.N): # loop over control intervals

            next_x = self.X_casadi[0,k] + self.U_casadi[0,k]*cos(self.X_casadi[2,k])*self.dt
            next_y = self.X_casadi[1,k] + self.U_casadi[0,k]*sin(self.X_casadi[2,k])*self.dt
            next_theta = self.X_casadi[2,k] + self.U_casadi[1,k]*self.dt

            self.opti.subject_to(self.X_casadi[0,k+1]==next_x) # close the gaps
            self.opti.subject_to(self.X_casadi[1,k+1]==next_y) # close the gaps
            self.opti.subject_to(self.X_casadi[2,k+1]==next_theta) # close the gaps   


        # Solver parameters ---------------------------------------
        p_opts = dict(print_time=False, verbose=False) 
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt",p_opts,s_opts) # set numerical backend



    def compute_control(self, initial_state, reference_x, reference_y):
        # Setting Initial State ---------------------------------------
        start_time = time.time()
        self.opti.subject_to(self.x_casadi[0]==initial_state[0])  
        self.opti.subject_to(self.y_casadi[0]==initial_state[1])   
        self.opti.subject_to(self.theta_casadi[0]==initial_state[2])

        # Setting Cost Function ---------------------------------------
        position_error = 0
        input_use = 0
    
        for k in range(1,self.N):
            ref_x = reference_x[k]
            ref_y = reference_y[k]
            position_error += self.Q*(self.x_casadi[k] - ref_x)@(self.x_casadi[k] - ref_x).T
            position_error += self.Q*(self.y_casadi[k] - ref_y)@(self.y_casadi[k] - ref_y).T
      
        for k in range(0,self.N-1):
            ref_v = 0
            ref_w = 0
            input_use += self.R*(self.U_casadi[0,k] - ref_v)@(self.U_casadi[0,k] - ref_v).T 
            input_use += self.R*(self.U_casadi[1,k] - ref_w)@(self.U_casadi[1,k] - ref_w).T 
        
        self.opti.minimize(position_error + input_use)
        #print("Initialization time: ", time.time()-start_time)
        
        # Compute solution ---------------------------------------
        start_time = time.time()
        sol = self.opti.solve()
        #print("Solving time: ", time.time()-start_time)

        # Taking just first action ---------------------------------------
        v = sol.value(self.U_casadi)[0]
        w = sol.value(self.U_casadi)[1]

        # For the next step! ---------------------------------------
        self.initialize_casadi() 
        return v[0],w[0]