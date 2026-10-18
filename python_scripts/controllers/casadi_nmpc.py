from casadi import * # type: ignore
import random 
import matplotlib.pyplot as plt # type: ignore
import numpy as np
import time

class Casadi_NMPC:
    """Class for a Nonlinear Model Predictive Control law based using Casadi 
    """
    def __init__(self,horizon, dt):
        """Init func
        Args:
            horizon (float): how many steps to look into the future
            input_constraint (np.array): control constraints
            state_constraints (np.array): state contraints
            dt (float): sampling time
        """
        self.N = horizon
        # same saturation applied by run_controllers.py
        self.v_max = 1.0
        self.w_max = 3.0

        self.n_actionsMPC = 2
        self.dt = dt
        self.numberState = 3

        self.Q_pos = 100
        self.Q_yaw = 0
        self.R = 0.001  # small: with a 0.2 s horizon a larger R prevents the correction of lateral errors

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
        self.yaw_casadi   = self.X_casadi[2,:]

        self.U_casadi = self.opti.variable(self.n_actionsMPC,self.N)   # control trajectory

        # Initial State Constraint -----------------------------------
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.opti.subject_to(self.x_casadi[0]==self.x_0)
        self.opti.subject_to(self.y_casadi[0]==self.y_0)
        self.opti.subject_to(self.yaw_casadi[0]==self.yaw_0) 

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
            next_x = self.X_casadi[0,k] + self.U_casadi[0,k]*cos(self.X_casadi[2,k])*self.dt
            next_y = self.X_casadi[1,k] + self.U_casadi[0,k]*sin(self.X_casadi[2,k])*self.dt
            next_theta = self.X_casadi[2,k] + self.U_casadi[1,k]*self.dt

            self.opti.subject_to(self.X_casadi[0,k+1]==next_x) # close the gaps
            self.opti.subject_to(self.X_casadi[1,k+1]==next_y) # close the gaps
            self.opti.subject_to(self.X_casadi[2,k+1]==next_theta) # close the gaps   

            
    def set_constraints(self):
        """Setting input constraints
        """
        for k in range(self.N): # loop over control intervals
            #linear velocity
            self.opti.subject_to(self.U_casadi[0,k] <= self.v_max)
            self.opti.subject_to(self.U_casadi[0,k] >= -self.v_max)
            #angular velocity
            self.opti.subject_to(self.U_casadi[1,k] <= self.w_max)
            self.opti.subject_to(self.U_casadi[1,k] >= -self.w_max)



    def set_cost_function(self):
        """Setting the cost function
        """

        # Parametric Cost Function -------------------------------------
        pose_error = 0
        input_use = 0
        
        self.reference_x = self.opti.parameter(self.N+1)
        self.reference_y = self.opti.parameter(self.N+1)
        self.reference_yaw = self.opti.parameter(self.N+1)
        self.reference_v = self.opti.parameter(self.N)
        self.reference_w = self.opti.parameter(self.N)


        # Predicted state k is compared with reference k (k = 0 is the actual state) ----------------
        for k in range(1, self.N+1):
            pose_error += self.Q_pos*(self.x_casadi[k] - self.reference_x[k])**2
            pose_error += self.Q_pos*(self.y_casadi[k] - self.reference_y[k])**2
            pose_error += self.Q_yaw*(self.yaw_casadi[k] - self.reference_yaw[k])**2

        # Inputs are pushed toward the velocities that follow the reference -------------------------
        for k in range(self.N):
            input_use += self.R*(self.U_casadi[0,k] - self.reference_v[k])**2
            input_use += self.R*(self.U_casadi[1,k] - self.reference_w[k])**2

        self.opti.minimize(pose_error + input_use)


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
        self.opti.set_value(self.yaw_0, initial_state[2])

        # Setting Reference ------------------------------------------
        ref_yaw = []
        ref_v = []
        yaw = initial_state[2]
        for k in range(0, self.N):
                ref_x_dot = (reference_x[k+1] - reference_x[k])/self.dt
                ref_y_dot = (reference_y[k+1] - reference_y[k])/self.dt
                ref_v.append(np.hypot(ref_x_dot, ref_y_dot))
                # repeated reference points (end of the path) have no heading: keep the previous one
                if(ref_v[-1] > 1e-3):
                    yaw = np.arctan2(ref_y_dot, ref_x_dot)
                ref_yaw.append(yaw)
        ref_yaw.append(ref_yaw[-1])
        ref_w = [np.arctan2(np.sin(ref_yaw[k+1] - ref_yaw[k]), np.cos(ref_yaw[k+1] - ref_yaw[k]))/self.dt for k in range(self.N)]

        self.opti.set_value(self.reference_yaw, ref_yaw)
        self.opti.set_value(self.reference_x, reference_x)
        self.opti.set_value(self.reference_y, reference_y)
        self.opti.set_value(self.reference_v, ref_v)
        self.opti.set_value(self.reference_w, ref_w)

        # Compute solution ---------------------------------------
        start_time = time.time()
        try:
            U = self.opti.solve().value(self.U_casadi)
        except RuntimeError:
            # Ipopt did not converge: use its last iterate instead of stopping the controller
            print("Casadi NMPC: solver did not converge, using the last iterate")
            U = self.opti.debug.value(self.U_casadi)

        # Taking just first action ---------------------------------------
        return U[0][0], U[1][0]