import sys
# Import planners ---------------------------------------
sys.path.append('/home/python_scripts/planners/')
from grid_based.a_star import A_star # type: ignore
from grid_based.greedy_best_first_search import Greedy_Best_First_Search # type: ignore
from grid_based.breadth_first_search import Breadth_First_Search # type: ignore 
from grid_based.djikstra import Djikstra # type: ignore

from sampling_based.rrt import RRT

# Import controllers ---------------------------------------
sys.path.append('/home/python_scripts/controllers')
from io_linearization import IO_linearization # type: ignore
from io_linearization_mpc import IO_linearization_MPC # type: ignore
from casadi_nmpc import Casadi_NMPC # type: ignore
from nonlinear_lyapunov import Nonlinear_lyapunov # type: ignore
from approximate_linearization import Approximate_linearization # type: ignore
from dynamic_linearization import Dynamic_linearization # type: ignore
from ilqr import iLQR 
from predictive_sampling import Sampling_MPC
sys.path.append('/home/python_scripts/controllers/acados')
from acados_nmpc import Acados_NMPC 


from scipy.interpolate import CubicSpline, Akima1DInterpolator
# Import robot model and path utilities ---------------------------------------
from robot_model import Robot
from path_utilities import interpolate_path, filter_map, draw_map # type: ignore

# Import python stuff ---------------------------------------
import matplotlib.pyplot as plt # type: ignore
from pgm_reader import Reader # type: ignore
import time
import copy
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)


print("Choose a controller:\n", 
    "1 - Casadi NMPC\n",
    "2 - IO Linearization\n",
    "3 - Dynamic Linearization\n",
    "4 - Nonlinear Lyapunov\n",
    "5 - Approximate Linearization\n",
    "6 - Acados NMPC\n",
    "7 - Iterative LQR\n",
    "8 - Sampling MPC\n",
    "9 - IO Linearization MPC\n")

controller_choice = input()
controller_choice = int(controller_choice)

# Inizial robot and some parameters ---------------------------------------
state_robot = np.array([0.0, 0.0, 0])  # x y theta
dt = 0.02                              # sampling time
robot = Robot(dt)

#  target course
ax = np.array([0.0, 10.0, 10.0, 5.0, 6.0])
ay = np.array([0.0, 0.0, -3.0, -2.0, 0.0])

time_ = np.arange(0, len(ax), 1)
spline_x = CubicSpline(time_,ax)
xs = np.arange(0, len(time_)-1, dt/10.)



time_ = np.arange(0, len(ay), 1)
spline_y = CubicSpline(time_, ay)
ys = np.arange(0, len(time_)-1, dt/10.)


path_spline_x = []
for i in range(len(xs)):
    temp = spline_x(xs[i])
    path_spline_x.insert(-1,temp)
path_spline_x = np.array(path_spline_x[0:len(path_spline_x)-1])

#print("path_spline_x", path_spline_x)

path_spline_y = []
for i in range(len(ys)):
    temp = spline_y(ys[i])
    path_spline_y.insert(-1,temp)
path_spline_y = np.array(path_spline_y[0:len(path_spline_y)-1])



# Control ---------------------------------------
horizon = 0
if(controller_choice==1):
    horizon = 10
    controller = Casadi_NMPC(horizon, dt)

elif(controller_choice==2):
    b = 0.1
    k1 = 3
    k2 = 3
    controller = IO_linearization(b,k1,k2, dt)

elif(controller_choice==3):
    k1 = 5
    k2 = 50
    controller = Dynamic_linearization(k1, k2, dt)

elif(controller_choice==4):
    k1 = 15
    k2 = 15
    k3 = 5
    controller = Nonlinear_lyapunov(k1=k1, k2=k2, k3=k3, dt=dt)

elif(controller_choice==5):
    k1 = 5
    k2 = 5
    k3 = 5
    controller = Approximate_linearization(k1=k1, k2=k2, k3=k3, dt=dt)

elif(controller_choice==6):
    horizon = 10
    controller = NMPC(horizon, dt)

elif(controller_choice==7):
    horizon = 20
    controller = iLQR(horizon=horizon, dt=dt)

elif(controller_choice==8):
    horizon = 20
    controller = Sampling_MPC(horizon=horizon,dt= dt, init_jax = True, linear = False)
    
elif(controller_choice==9):
    b = 0.1
    horizon = 20
    controller = IO_linearization_MPC(horizon, b=b, dt=dt)





controller.reset()
state_evolution = [copy.copy(state_robot)]
for j in range(np.shape(path_spline_x)[0]):
    print("State_robot: ", state_robot)
    state_evolution = np.append(state_evolution, [copy.copy(state_robot)], axis=0)

    start_time = time.time()

    reference_x = []
    reference_y = []
    if(horizon == 0):
        reference_x = path_spline_x[j]
        reference_y = path_spline_y[j]
    else:
        for i in range(horizon+1):
            if(j+i < np.shape(path_spline_x)[0]):
                reference_x.append(path_spline_x[j+i])
                reference_y.append(path_spline_y[j+i])
                #reference_x.append(0)
                #reference_y.append(0)
            else:
                reference_x.append(path_spline_x[-1])
                reference_y.append(path_spline_y[-1])
                #reference_x.append(0)
                #reference_y.append(0)

    v, w = controller.compute_control(state_robot, reference_x, reference_y)
    
    print("Control actions: ", [v, w])
    print("Control time: ", time.time()-start_time)
    state_robot = robot.integrate(state_robot, v, w)
    print("##############################")
    

# Plotting ---------------------------------------
state_evolution = np.array(state_evolution)
plt.figure("Tracking Performance") 
plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

plt.subplot(211)
plt.plot(state_evolution[:,0])
plt.plot(path_spline_x[:])
plt.ylabel('x')
plt.xlabel('t')
plt.legend(['robot', 'path'])

plt.subplot(212)
plt.plot(state_evolution[:,1])
plt.plot(path_spline_y[:])
plt.ylabel('y')
plt.xlabel('t')
plt.show(block=False)


plt.figure("Tracking Performance x-y") 
plt.plot(state_evolution[:,0],state_evolution[:,1])
plt.plot(path_spline_x[:],path_spline_y[:])
plt.ylabel('y')
plt.ylabel('x')
plt.show()

