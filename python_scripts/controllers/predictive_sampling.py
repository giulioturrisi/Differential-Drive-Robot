import jax
import jax.numpy as jnp
from jax import jit
from jax import random
import os

print("jax.default_backend()", jax.default_backend())
print("ax.devices()", jax.devices())
#os.environ["XLA_PYTHON_CLIENT_ALLOCATOR"] = "platform"
#os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = ".75"
#jax.config.update('jax_platform_name', 'cpu')

gpu_device = jax.devices('gpu')[0]
cpu_device = jax.devices('cpu')[0]

import numpy as np
import matplotlib.pyplot as plt #
import time

import copy 
import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot

class Sampling_MPC:
    """This is a small class that implements a sampling based control law"""


    def __init__(self, horizon = 200, dt = 0.01, num_computations = 10000, init_jax = True, linear = True, device="gpu"):
        """
        Args:
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        self.horizon = horizon
        self.dt = dt
        self.state_dim = 3
        self.control_dim = 2
        self.num_computations = num_computations

        if(device=="gpu"):
            self.device = gpu_device
        else:
            self.device = cpu_device
        
        if(linear == True):
            self.spline_fun = jax.jit(self.compute_linear_spline, device=self.device)
        else:
            self.spline_fun = jax.jit(self.compute_cubic_spline, device=self.device) 

        self.robot = Robot(self.dt)

        self.Q = jnp.identity(self.state_dim)
        self.Q.at[0,0].set(0.0)
        self.Q.at[1,1].set(0.0)
        self.Q.at[2,2].set(0.0)

        

        self.R = jnp.identity(self.control_dim)

        self.num_parameters = 16

        # the first call of jax is very slow, hence we should do this since the beginning! ------------------
        if(init_jax):
            vectorized_forward_sim = jax.vmap(self.compute_forward_simulations, in_axes=(0,0,0), out_axes=0)
            self.jit_vectorized_forward_sim = jax.jit(vectorized_forward_sim, device=self.device)
            
            threads = self.num_computations
            
            reference_x = []
            reference_y = []
            reference_theta = []
            for i in range(self.horizon):
                    reference_x.append(0.0)
                    reference_y.append(0.0)
                    reference_theta.append(0.0)
            state_des = jnp.column_stack([reference_x, reference_y, reference_theta])
            xs_des = jnp.tile(state_des, (self.num_computations,1)).reshape(self.num_computations, self.horizon, self.state_dim)
            xs = jnp.zeros((self.state_dim*threads,)).reshape(threads,self.state_dim)
            
            key = random.PRNGKey(42)
            parameters_map = random.randint(key,(self.num_parameters*threads,), minval=-200, maxval=200 )/100.
            self.parameters_map = parameters_map.reshape(threads,self.num_parameters)
            
            self.jit_vectorized_forward_sim(xs, xs_des, self.parameters_map)

            
    def reset(self,):
        """Every control class should have a reset function
        """
        return
    
    
    def compute_linear_spline(self, parameters, step):
        index = 0
        index = jax.numpy.where(step > self.horizon/4, 2, index)
        index = jax.numpy.where(step > self.horizon/2, 4, index)
        index = jax.numpy.where(step > self.horizon-10, 6, index) 

        q = (step*0.01 - 0)/(self.horizon*0.01)
        v = (1-q)*parameters[index+0] + q*parameters[index+1]
        w = (1-q)*parameters[index+8] + q*parameters[index+9]

        return v, w
    
    def compute_cubic_spline(self, parameters, step):
        
        q = (step*0.01 - 0)/(self.horizon*0.01)
        
        phi = (1./2.)*(((parameters[2] - parameters[1])/0.5) + ((parameters[1] - parameters[0])/0.5))
        phi_next = (1./2.)*(((parameters[3] - parameters[2])/0.5) + ((parameters[2] - parameters[1])/0.5))
        
        a_v = 2*q*q*q - 3*q*q + 1
        b_v = (q*q*q - 2*q*q + q)*0.5
        c_v = -2*q*q*q + 3*q*q
        d_v = (q*q*q - q*q)*0.5
        v = a_v*parameters[1] + b_v*phi + c_v*parameters[2] + d_v*phi_next

        phi = (1./2.)*(((parameters[6] - parameters[5])/0.5) + ((parameters[5] - parameters[4])/0.5))
        phi_next = (1./2.)*(((parameters[7] - parameters[6])/0.5) + ((parameters[6] - parameters[5])/0.5))
        
        a_w = 2*q*q*q - 3*q*q + 1
        b_w = (q*q*q - 2*q*q + q)*0.5
        c_w = -2*q*q*q + 3*q*q
        d_w = (q*q*q - q*q)*0.5
        w = a_w*parameters[5] + b_w*phi + c_w*parameters[6] + d_w*phi_next
       
        return v, w


    def compute_forward_simulations(self, initial_state, state_des, parameters):
        """Calculate cost of a rollout of the dynamics given random parameters
        Args:
            initial_state (np.array): actual state of the robot
            state_des (np.array): desired state of the robot
            parameters (np.array): parameters for the controllers
        Returns:
            (float): cost of the rollout
        """
        state = initial_state
        cost = 0.0
    
        def iterate_fun(n, carry):
            cost, state, state_des = carry

            v, w = self.spline_fun(parameters, n)
            #v, w = self.compute_linear_spline(parameters, n)
            #v, w = self.compute_cubic_spline(parameters, n)

            v = jax.numpy.where(v > 2.0, 2.0, v)
            v = jax.numpy.where(v < -2.0, -2.0, v)
            
            w = jax.numpy.where(w > 2.0, 2.0, w)
            w = jax.numpy.where(w < -2.0, -2.0, w)

            state_next = self.robot.integrate_jax(state.reshape(self.state_dim,), v, w);
            
            error = state_next.reshape(self.state_dim, 1) - state_des[n].reshape(self.state_dim, 1)
            cost_next = error[0]*1.0*error[0] + error[1]*1.0*error[1]
            cost_next = [cost_next]

            return (cost_next[0][0] + cost, state_next, state_des)

        carry = (cost, state, state_des)
        cost, state, state_des = jax.lax.fori_loop(0, self.horizon, iterate_fun, carry)
    
        '''for n in range(0, self.horizon):
            
            index = 0
            index = jax.numpy.where(n > self.horizon/4, 2, index)
            index = jax.numpy.where(n > self.horizon/2, 4, index)
            index = jax.numpy.where(n > self.horizon-10, 6, index)
          

            q = (n*0.01 - 0)/(self.horizon*0.01)
            v = (1-q)*parameters[index+0] + q*parameters[index+1]
            w = (1-q)*parameters[index+8] + q*parameters[index+9]
            

            v = jax.numpy.where(v > 2.0, 2.0, v)
            v = jax.numpy.where(v < -2.0, -2.0, v)
            
            w = jax.numpy.where(w > 2.0, 2.0, w)
            w = jax.numpy.where(w < -2.0, -2.0, w)

            
            #print("state_des inside", state_des)
            #print("state_des inside[n]", state_des[n])
            #state_des_single = jnp.array([state_des[n][0], state_des[n][1], 0.0])
            #print("state_des_single", state_des_single.reshape(3,1))
            state = self.robot.integrate_jax(state.reshape(self.state_dim,), v, w)
            error = state.reshape(self.state_dim,1) - state_des[n].reshape(self.state_dim,1)
            

            cost_next = error[0]*1.0*error[0] + error[1]*1.0*error[1] + error[2]*0.001*error[2]*0.0
            cost_next = [cost_next]
            #cost_next = (state.reshape(self.state_dim,1) - state_des.reshape(self.state_dim,1)).T@self.Q@(state.reshape(self.state_dim,1) - state_des.reshape(self.state_dim,1))
            
            
            #state = state_next
            cost = cost + cost_next[0][0]'''
        
        return cost
    
    
    def compute_control(self, state, reference_x, reference_y):
        """Compute control inputs
        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state
        Returns:
            (np.array): optimized control inputs
        """
        reference_theta = []
        for i in range(self.horizon):
            reference_theta.append(0.0)
        reference_x.pop(-1)
        reference_y.pop(-1)
        
        state_vec = jnp.tile(state, (self.num_computations,1))
        state_des = jnp.column_stack([reference_x, reference_y, reference_theta])
        state_des_vec = jnp.tile(state_des, (self.num_computations,1)).reshape(self.num_computations, self.horizon, self.state_dim)
        
        #time_start = time.time()
        #key = random.PRNGKey(42)
        #parameters_map = random.randint(key,(self.num_parameters*1,), minval=-200, maxval=200 )/100.

        
        time_start = time.time()
        cost = self.jit_vectorized_forward_sim(state_vec, state_des_vec, self.parameters_map)
        #print("cost computation time: ", time.time()-time_start)

        best_index = jnp.nanargmin(cost)
        best_parameters = self.parameters_map[best_index]
        #time_start = time.time()
        v, w = self.spline_fun(best_parameters, 0)
        print("computation time: ", time.time()-time_start)

        return np.float64(v), np.float64(w)




if __name__=="__main__":
    control = Sampling_MPC(dt=0.01, horizon=20, init_jax = True, num_computations = 1000)

    x = jnp.array([0.0, 0.0, 0.0])
    x_des = jnp.array([1.0, -0.7, 0.0])
    reference_x = []
    reference_y = []
    reference_theta = []
    for i in range(control.horizon):
            reference_x.append(x_des[0])
            reference_y.append(x_des[1])
            reference_theta.append(0.0)
    state_des = jnp.column_stack([reference_x, reference_y, reference_theta])

    #threads = 2
    xs = jnp.tile(x, (control.num_computations,1)).reshape(control.num_computations, control.state_dim)
    xs_des = jnp.tile(state_des, (control.num_computations,1)).reshape(control.num_computations, control.horizon, control.state_dim)

    key = random.PRNGKey(42)
    parameters_map = random.randint(key,(control.num_parameters*control.num_computations,), minval=-200, maxval=200 )/100.0
    parameters_map = parameters_map.reshape(control.num_computations,control.num_parameters)

  

    # single computation test ------------------------------------
    '''start_time = time.time()
    cost = control.compute_forward_simulations(xs[0], xs_des[0], parameters_map[0])
    print("non-compiled single jax: ", time.time()-start_time)

    v_fd = jax.vmap(control.compute_forward_simulations, in_axes=(0,0,0), out_axes=0)
    start_time = time.time()
    cost = v_fd(xs, xs_des, parameters_map)
    print("non-compiled multi jax: ", time.time()-start_time)
    

    #start_time = time.time()
    #jit_fd = jax.jit(control.compute_forward_simulations)
    #print("compilation jax: ", time.time()-start_time)
    
    #start_time = time.time() 
    #cost = jit_fd(x, x_des, parameters[0])
    #print("compiled jax: ", time.time()-start_time)
    #print("cost: ", cost)
    
    #start_time = time.time()
    #cost = jit_fd(x, x_des, parameters[0])
    #print("compiled jax single: ", time.time()-start_time)
    

    # parallel computation test ------------------------------------
    threads = 2
    xs = jnp.tile(x, (threads,1)).reshape(threads,control.state_dim)
    xs_des = jnp.tile(x_des, (threads,1)).reshape(threads,control.state_dim)

    
    key = random.PRNGKey(42)
    parameters_map = random.randint(key,(control.num_parameters*threads,), minval=-200, maxval=200 )/100.0
    parameters_map = parameters_map.reshape(threads,control.num_parameters)

  
    
    v_fd = jax.vmap(control.compute_forward_simulations, in_axes=(0,0,0), out_axes=0)
    start_time = time.time()
    cost = v_fd(xs, xs_des, parameters_map)
    #print("non compiled VMAP jax: ", time.time()-start_time)
    #print("costs_out", cost)
    #print("minimum cost", np.nanmin(cost))
    min_cost_index = np.nanargmin(cost)
    #print("minimum cost index", min_cost_index)
    #print("best parameters", parameters_map[min_cost_index])'''
    
    #start_time = time.time()
    '''jit_v_fd = jax.jit(v_fd)
    #print("parallel compilation jax: ", time.time()-start_time)

    #start_time = time.time()
    costs = jit_v_fd(xs, xs_des, parameters_map)
    #print("parallel compiled jax: ", time.time()-start_time)
    #print("costs", costs)

    start_time = time.time()
    costs = jit_v_fd(xs, xs_des, parameters_map)
    #print("costs_out", costs)
    #print("minimum cost", np.nanmin(costs))
    #print("parallel VMAP jax: ", time.time()-start_time)
    
    x = jnp.array([0, 0.2, 0.4])
    x_des = jnp.array([0, 0.0, 0])
    xs = jnp.tile(x, (threads,1)).reshape(threads,3)    
    xs_des = jnp.tile(x_des, (threads,1)).reshape(threads,3)
    start_time = time.time()
    costs = jit_v_fd(xs, xs_des, parameters_map)
    #print("costs_out", costs)
    print("minimum cost", np.nanmin(costs))
    print("parallel VMAP jax changing init state: ", time.time()-start_time)'''
    
 
    '''start_time = time.time()
    for i in range(0,threads):
        parameters = np.random.rand(control.num_parameters,1).reshape(-1,control.num_parameters)
        #cost = jit_fd(x, x_des, parameters[i])
        cost = control.compute_forward_simulations(x, x_des, parameters[0])
        #cost = control.compute_forward_simulation(x, x_des)
        print(cost)
    print("non parallel compiled jax: ", time.time()-start_time)'''

    
    
    

    state_evolution = [copy.copy(x)]
    for j in range(0,1000):
        print("State_robot: ", x)
        state_evolution = np.append(state_evolution, [copy.copy(x)], axis=0)

        start_time = time.time()

        reference_x = []
        reference_y = []
        reference_theta = []
        for i in range(control.horizon):
                reference_x.append(x_des[0])
                reference_y.append(x_des[1])
                reference_theta.append(0.0)

        v, w = control.compute_control(x, reference_x, reference_y, reference_theta)
        
        print("Control actions: ", [v, w])
        print("Control time: ", time.time()-start_time)
        x = control.robot.integrate_jax(x, v, w)
        print("##############################")


    plt.figure("Tracking Performance x-y") 
    plt.plot(state_evolution[:,1],state_evolution[:,0])
    #plt.plot(path_spline_x[:],path_spline_y[:])
    plt.ylabel('y')
    plt.xlabel('x')
    plt.show()