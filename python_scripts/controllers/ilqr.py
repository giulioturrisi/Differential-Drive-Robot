import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot

import matplotlib.pyplot as plt # type: ignore

import time

import copy

import casadi as cs

class iLQR:
    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        self.lin_state = np.zeros(3)
        self.lin_tau = np.zeros(2)
        self.horizon = horizon
        self.iteration = 5
        self.dt = dt

        self.ddrive = Robot(self.dt)

        self.state_dim = 3
        self.control_dim = 2

        self.Q = np.identity(3)*100
        self.Q[2,2] = 10
        

        

        self.R = np.identity(2)*10
        
        # compute optimal cost to go LQR
        #self.P = self.compute_discrete_LQR_P(self.lin_state, self.lin_tau)

        self.P = np.zeros((self.state_dim, self.state_dim))
        self.P_vec = np.zeros((self.horizon+1, self.state_dim, self.state_dim));
        self.P_vec[self.horizon] = self.P
        self.V_vec = np.zeros(((self.horizon+1),self.state_dim,1));
    

        self.state_vec = np.zeros(((self.horizon+1), self.state_dim, 1));
        self.control_vec = np.zeros(((self.horizon), self.control_dim, 1));


        self.A_vec = np.zeros((self.horizon, self.state_dim, self.state_dim));
        self.B_vec = np.zeros((self.horizon, self.state_dim, self.control_dim));

        self.Q_uu_vec = np.zeros((self.horizon, self.control_dim, self.control_dim));
        self.pinv_Q_uu_vec = np.zeros((self.horizon, self.control_dim, self.control_dim));
        self.Q_ux_vec = np.zeros((self.horizon, self.control_dim, self.state_dim));
        self.Q_u_vec = np.zeros((self.horizon, self.control_dim, 1));

        #self.twip.fd.generate("fd.c")
        #C = cs.Importer("fd.c", "shell")
        #self.f = cs.external('fd',C)


    def compute_discrete_LQR_P(self,lin_state, lin_tau):

        dt = 0.001

        P_next = np.identity(self.state_dim)
        
        A = self.ddrive.A_f(lin_state, lin_tau)
        B = self.ddrive.B_f(lin_state, lin_tau)       

        A_discrete = A*dt + np.identity(self.state_dim)
        B_discrete = B*dt


        for i in range(0, 2000):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
            self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        return P_next
    
    def compute_backward_pass(self, state_des_vec):
        #print("##BACKWATD PASS")
        

        for step in range(0,self.horizon):
            state_actual = self.state_vec[self.horizon-step-1];
            state_des = state_des_vec[self.horizon-step-1]
            control_actual = self.control_vec[self.horizon-step-1];
            u = control_actual 

            # compute discrete A and B matrices
            A_step = self.ddrive.A_f(state_actual, control_actual)
            A_step = np.array(A_step)
            B_step = self.ddrive.B_f(state_actual, control_actual)
            B_step = np.array(B_step)

            A_step = A_step*self.dt + np.identity(self.state_dim)
            B_step = B_step*self.dt

            #temp = np.linalg.pinv((np.identity(self.state_dim) - A_step*self.dt/2.0))
            #A_step = (np.identity(self.state_dim) + A_step*self.dt/2.0)@temp
            #B_step = temp@B_step*np.sqrt(self.dt)

            #temp = np.linalg.pinv(np.identity(self.state_dim) - A_step*self.dt)
            #A_step = temp@(B_step*self.dt)
            #B_step = temp
            self.A_vec[self.horizon - step - 1] = A_step
            self.B_vec[self.horizon - step - 1] = B_step

    
            # calculate P - optimal cost to go, also known as V_xx
            V_xx = self.P_vec[self.horizon - step] #V_xx
            V_x = self.V_vec[self.horizon - step] #V_x
            
            # calculate Q_xx, Q_uu and pinv_Q_uu
            Q_xx = self.Q + A_step.T@V_xx@A_step
            Q_uu = self.R + B_step.T@V_xx@B_step
            pinv_Q_uu = np.linalg.pinv(Q_uu)

        
            # calculate Q_ux and Q_xu
            Q_ux = B_step.T@V_xx@A_step
            Q_xu = A_step.T@V_xx@B_step
            
            # calculate Q_u, Qx
            Q_u = (self.R@u) + B_step.T@V_x
            Q_x = self.Q@(state_actual - state_des) + A_step.T@V_x


            k = -pinv_Q_uu@Q_u
            K = -pinv_Q_uu@Q_ux
            
            V_x = Q_x - K.T@Q_uu@k
            V_xx = Q_xx - K.T@Q_uu@K

            # save everything
            self.Q_uu_vec[self.horizon - step - 1] = Q_uu
            self.pinv_Q_uu_vec[self.horizon - step - 1] = pinv_Q_uu
            self.Q_ux_vec[self.horizon - step - 1] = Q_ux
            self.Q_u_vec[self.horizon-step-1] = Q_u

            self.P_vec[self.horizon - step - 1] = V_xx
            self.V_vec[self.horizon - step - 1] = V_x
        

        


    def compute_forward_pass(self,initial_state):

        self.state_vec[0] = initial_state.reshape(self.state_dim,1) 
        state_forward = copy.deepcopy(self.state_vec)
        
        for step in range(0,self.horizon):

            start_time = time.time()
            
            #error = (self.state_vec[step] - state_forward[step])
            error = (self.state_vec[step] - state_forward[step])

            # taking value from backward pass
            pinv_Q_uu = self.pinv_Q_uu_vec[step]
            Q_ux = self.Q_ux_vec[step]
            Q_u = self.Q_u_vec[step]


            # new control update
            k = -pinv_Q_uu@Q_u
            K = -pinv_Q_uu@Q_ux


            
            self.control_vec[step,0] += k[0]*1 + K[0]@(error)
            self.control_vec[step,1] += k[1]*1 + K[1]@(error)
            
            
            #print("forward time first: ", time.time()-start_time)
            start_time = time.time()

            # simulate system
            state = self.state_vec[step]
            state = state.reshape(self.state_dim,)
            control = self.control_vec[step]
            control = control.reshape(self.control_dim,)


            #print("forward time second: ", time.time()-start_time)


            # integration
            #print("state forward", state)
            self.state_vec[step+1] = self.ddrive.integrate(state, control[0], control[1]).reshape(self.state_dim,1)

            
            
            '''#cost
            cost_temp = cost_temp + error'*Q*error + [u_l(1,step);u_r(1,step)]'*R*[u_l(1,step);u_r(1,step)];'''

        #print("evolution forward pass", self.state_vec)


    def compute_forward_simulation(self,initial_state):

        self.state_vec[0] = initial_state.reshape(self.state_dim,1) 

        for step in range(0,self.horizon):

            # simulate system
            state = self.state_vec[step]
            state = state.reshape(self.state_dim,)
            control = self.control_vec[step]
            control = control.reshape(self.control_dim,)
            #qdd = self.twip.fd(state,control); 
            #qdd = self.f(state,control); 
            #qdd = qdd[3:6]

            # integration
            self.state_vec[step+1] = self.ddrive.integrate(state, control[0], control[1]).reshape(self.state_dim,1)

            


    def compute_control(self, state, reference_x, reference_y):

        state_des_vec = np.zeros((self.horizon+1,3,1))

        ref_yaw = 0
        for k in range(0,self.horizon):
            ref_x = reference_x[k]
            ref_y = reference_y[k]
            
            ref_x_d = (reference_x[k+1] - ref_x)/self.dt
            ref_y_d = (reference_y[k+1] - ref_y)/self.dt
            ref_yaw = np.arctan2(ref_y_d, ref_x_d) 
            
            state_des_vec[k] = np.array([ref_x , ref_y, ref_yaw]).reshape(self.state_dim,1)


        ref_x = reference_x[self.horizon]
        ref_y = reference_y[self.horizon]    
        state_des_vec[self.horizon] = np.array([ref_x , ref_y, ref_yaw]).reshape(self.state_dim,1)

        print("state_des_vec", state_des_vec)

        


        # setting last V and initial system simulation #error!!!!
        #self.V_vec[self.horizon] = self.P@(state.reshape(self.state_dim,1) - state_des_vec[self.N].reshape(self.state_dim,1))
        self.compute_forward_simulation(initial_state=state)
        self.V_vec[self.horizon] = self.P@(self.state_vec[self.horizon].reshape(self.state_dim,1) - state_des_vec[self.horizon].reshape(self.state_dim,1))

        #plt.plot(self.state_vec[:,1])
        #plt.show()
        # compute control and gain
        for i in range(0,self.iteration):
            start_time = time.time()
            self.compute_backward_pass(state_des_vec)
            #print("backward time: ", time.time()-start_time)
            start_time = time.time()
            self.compute_forward_pass(initial_state=state.reshape(self.state_dim,1))
            #print("forward time: ", time.time()-start_time)
        
            #plt.plot(self.state_vec[:,1])
            #plt.show()
        #print("control vec", self.control_vec)

        return np.asscalar(self.control_vec[0,0]),np.asscalar(self.control_vec[0,1])





if __name__=="__main__":
    controller=iLQR(dt = 0.01, horizon=10)
    state = np.zeros(3)
    state[1] = -1
    state[2] = 0.001 #yaw
    state_des = np.zeros((controller.horizon,3,1))
    
    start_time = time.time()
    controller.compute_control(state, reference_x=state_des[:,0], reference_y=state_des[:,1])
    print("Control time: ", time.time()-start_time)

    plt.plot(controller.state_vec[:,1])
    plt.show()
