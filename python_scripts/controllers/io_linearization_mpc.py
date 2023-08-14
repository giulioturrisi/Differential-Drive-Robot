from casadi import * # type: ignore
import random 
import matplotlib.pyplot as plt # type: ignore
import numpy as np
import time
import math

class IO_linearization_MPC:
    """Class for a Nonlinear Model Predictive Control law based using Casadi 
    """
    def __init__(self,horizon, b, dt):
        """Init func
        Args:
            horizon (float): how many steps to look into the future
            input_constraint (np.array): control constraints
            state_constraints (np.array): state contraints
            dt (float): sampling time
        """
        self.N = horizon
        self.acc_max = 0.5

        self.b = b
        

        self.n_actionsMPC = 2
        self.dt = dt
        self.numberState = 2

        self.Q_pos = 100
        self.R = 0.1

        self.initialize_casadi()


    def reset(self,):
        """Every control class should have a reset function
        """
        return


    def initialize_casadi(self):
        """Initialize the casadi optimal control problem
        """

        # Casadi problem formulation ---------------------------------------
        self.opti = Opti() # Optimization problem
        
        # Decision variables ---------------------------------------
        self.X_casadi = self.opti.variable(self.numberState,self.N+1) # state trajectory
        self.x_casadi   = self.X_casadi[0,:]
        self.y_casadi   = self.X_casadi[1,:]
        

        self.U_casadi = self.opti.variable(self.n_actionsMPC,self.N)   # control trajectory

        # Initial State Constraint -----------------------------------
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        
        self.opti.subject_to(self.x_casadi[0]==self.x_0)
        self.opti.subject_to(self.y_casadi[0]==self.y_0)
        

        # State Constraints and Cost Function -------------------------
        self.set_kinematics()
        self.set_constraints()
        self.set_cost_function()
        
        # Solver parameters ---------------------------------------
        p_opts = dict(print_time=False, verbose=False) 
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt",p_opts,s_opts) # set numerical backend


    def set_kinematics(self):
        """Setting the kinematics constraints
        """
        # Kynematic Constraints ---------------------------------------
        for k in range(self.N): # loop over control intervals
            next_x = self.X_casadi[0,k] + self.U_casadi[0,k]*self.dt
            next_y = self.X_casadi[1,k] + self.U_casadi[1,k]*self.dt
            
            self.opti.subject_to(self.X_casadi[0,k+1]==next_x) # close the gaps
            self.opti.subject_to(self.X_casadi[1,k+1]==next_y) # close the gaps
            
            
    def set_constraints(self):
        """Setting input constraints
        """
        for k in range(self.N): # loop over control intervals
            #linear velocity
            self.opti.subject_to(self.U_casadi[0,k] <= self.acc_max)
            self.opti.subject_to(self.U_casadi[0,k] >= -self.acc_max)
            #angular velocity
            self.opti.subject_to(self.U_casadi[1,k] <= self.acc_max)
            self.opti.subject_to(self.U_casadi[1,k] >= -self.acc_max)



    def set_cost_function(self):
        """Setting the cost function
        """

        # Parametric Cost Function -------------------------------------
        pose_error = 0
        input_use = 0
        
        self.reference_x = self.opti.parameter(self.N+1)
        self.reference_y = self.opti.parameter(self.N+1)
        


        for k in range(1, self.N):
            ref_x = self.reference_x[k-1]
            ref_y = self.reference_y[k-1]
            

            pose_error += self.Q_pos*(self.x_casadi[k] - ref_x)@(self.x_casadi[k] - ref_x).T
            pose_error += self.Q_pos*(self.y_casadi[k] - ref_y)@(self.y_casadi[k] - ref_y).T
            
            ref_u1 = 0
            ref_u2 = 0

            input_use += self.R*(self.U_casadi[0,k] - ref_u1)@(self.U_casadi[0,k] - ref_u1).T 
            input_use += self.R*(self.U_casadi[1,k] - ref_u2)@(self.U_casadi[1,k] - ref_u2).T 
        
        
        # Last step N horizon -----------------------------------------------------------------------
        ref_x = self.reference_x[self.N]
        ref_y = self.reference_y[self.N]
        pose_error += self.Q_pos*(self.x_casadi[self.N] - ref_x)@(self.x_casadi[self.N] - ref_x).T
        pose_error += self.Q_pos*(self.y_casadi[self.N] - ref_y)@(self.y_casadi[self.N] - ref_y).T
        
        self.opti.minimize(pose_error + input_use*0)


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
        self.opti.set_value(self.x_0, initial_state[0])
        self.opti.set_value(self.y_0, initial_state[1])
        
        # Setting Reference ------------------------------------------
        for k in range(0, self.N):
                ref_x_dot = (reference_x[k+1] - reference_x[k])/self.dt
                ref_y_dot = (reference_y[k+1] - reference_y[k])/self.dt
          

        self.opti.set_value(self.reference_x, reference_x)
        self.opti.set_value(self.reference_y, reference_y)
           
        # Compute solution ---------------------------------------
        start_time = time.time()
        sol = self.opti.solve()
       
        # Taking just first action ---------------------------------------
        u1_io = sol.value(self.U_casadi)[0]
        u2_io = sol.value(self.U_casadi)[1]

        state_yaw = initial_state[2]
        v = math.cos(state_yaw)*u1_io + math.sin(state_yaw)*u2_io;
        w = (-math.sin(state_yaw)*u1_io/self.b) + (math.cos(state_yaw)*u2_io/self.b);


        return v[0], w[0]