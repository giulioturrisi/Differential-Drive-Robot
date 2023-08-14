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
from predictive_sampling import Sampling_MPC


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

# Inizial robot and some parameters ---------------------------------------
state_robot = np.array([0.0, 0.0, 0.0])  # x y theta
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
horizon = 20
controller = Sampling_MPC(horizon=horizon,dt= dt, init_jax = True, linear = False)


controller.reset()
state_evolution = [copy.copy(state_robot)]
for j in range(np.shape(path_spline_x)[0]):
    print("State_robot: ", state_robot)
    state_evolution = np.append(state_evolution, [copy.copy(state_robot)], axis=0)

    start_time = time.time()

    reference_x = []
    reference_y = []
    reference_theta = []
    for i in range(horizon+1):
        if(j+i < np.shape(path_spline_x)[0]):
            reference_x.append(path_spline_x[j+i])
            reference_y.append(path_spline_y[j+i])
            reference_theta.append(path_spline_y[j+i])
        else:
            reference_x.append(path_spline_x[-1])
            reference_y.append(path_spline_y[-1])
            reference_theta.append(path_spline_y[-1])

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

