from casadi import * # type: ignore
import random 
import matplotlib.pyplot as plt # type: ignore
import numpy as np
import time

class Casadi_nmpc:
    """Class for a Nonlinear Model Predictive Control law based using Casadi 
    """
    def __init__(self,horizon, input_constraint, state_constraints, dt):
        """Init func
        Args:
            horizon (float): how many steps to look into the future
            input_constraint (np.array): control constraints
            state_constraints (np.array): state contraints
            dt (float): sampling time
        """
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
        """Every time we want to compute a solution we initialize the problem
        """
        # Casadi problem formulation ---------------------------------------
        self.opti = Opti() # Optimization problem
        
        # Decision variables ---------------------------------------
        self.X_casadi = self.opti.variable(self.numberState,self.N+1) # state trajectory
        self.x_casadi   = self.X_casadi[0,:]
        self.y_casadi   = self.X_casadi[1,:]
        self.yaw_casadi   = self.X_casadi[2,:]
            
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
        """Compute the control actions
        Args:
            initial_state (np.array): actual state of the robot
            reference_x (np.array): x reference for the robot
            reference_y (np.array): y reference for the robot
        Returns:
            (np.array): control actions
        """
        # Setting Initial State ---------------------------------------
        start_time = time.time()
        self.opti.subject_to(self.x_casadi[0]==initial_state[0])  
        self.opti.subject_to(self.y_casadi[0]==initial_state[1])   
        self.opti.subject_to(self.yaw_casadi[0]==initial_state[2])

        # Setting Cost Function ---------------------------------------
        position_error = 0
        input_use = 0
        ref_yaw = 0

        
        # Fill cost function using flat outputs ---------------------------------------------------
        for k in range(1,self.N):
            ref_x = reference_x[k]
            ref_y = reference_y[k]

            ref_x_dot = (reference_x[k+1] - ref_x)/self.dt
            ref_y_dot = (reference_y[k+1] - ref_y)/self.dt
            ref_yaw = np.arctan2(ref_y_dot, ref_x_dot) 

            position_error += self.Q*(self.x_casadi[k] - ref_x)@(self.x_casadi[k] - ref_x).T
            position_error += self.Q*(self.y_casadi[k] - ref_y)@(self.y_casadi[k] - ref_y).T
            position_error += self.Q*(self.yaw_casadi[k] - ref_yaw)@(self.yaw_casadi[k] - ref_yaw).T

            
            ref_x_ddot = (((reference_x[k+2] - reference_x[k+1])/self.dt ) - ref_x_dot)/self.dt
            ref_y_ddot = (((reference_y[k+2] - reference_y[k+1])/self.dt ) - ref_y_dot)/self.dt
            
            ref_v = np.sqrt(ref_x_dot*ref_x_dot + ref_y_dot*ref_y_dot)
            ref_w = (ref_y_ddot*ref_x_dot - ref_x_ddot*ref_y_dot)/(ref_x_dot*ref_x_dot + ref_y_dot*ref_y_dot)
            
            input_use += self.R*(self.U_casadi[0,k] - ref_v)@(self.U_casadi[0,k] - ref_v).T 
            input_use += self.R*(self.U_casadi[1,k] - ref_w)@(self.U_casadi[1,k] - ref_w).T 
        
        
        # Last step N horizon -----------------------------------------------------------------------
        ref_x = reference_x[self.N]
        ref_y = reference_y[self.N]
        position_error += self.Q*(self.x_casadi[self.N] - ref_x)@(self.x_casadi[self.N] - ref_x).T
        position_error += self.Q*(self.y_casadi[self.N] - ref_y)@(self.y_casadi[self.N] - ref_y).T
        position_error += self.Q*(self.yaw_casadi[self.N] - ref_yaw)@(self.yaw_casadi[self.N] - ref_yaw).T
        
        
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