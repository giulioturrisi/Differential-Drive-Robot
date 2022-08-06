import sys
# Import planners ---------------------------------------
sys.path.append('/home/python_simulation/planners/')
from grid_based.a_star import A_star # type: ignore
from grid_based.greedy_best_first_search import Greedy_Best_First_Search # type: ignore
from grid_based.breadth_first_search import Breadth_First_Search # type: ignore 
from grid_based.djikstra import Djikstra # type: ignore

# Import controllers ---------------------------------------
sys.path.append('/home/python_simulation/controllers')
from io_linearization import IO_linearization # type: ignore
from casadi_nmpc import Casadi_nmpc # type: ignore

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


# Read a map ---------------------------------------
f = './maps/map.pgm'
reader = Reader()
image = reader.read_pgm(f)
width = reader.width
height = reader.height



# Inizial robot and some parameters ---------------------------------------
state_robot = np.array([15, 2.45, 0])  # x y theta
dt = 0.02                               # sampling time
robot = Robot(dt)

#draw_map(image, state_robot, goal, resolution)


# Plan ---------------------------------------
goal = np.array([5, 18])             
max_iteration = 1000                 
map_resolution = 0.05
visualize = True

planner = A_star(state_robot, goal, image, map_resolution)
#planner = Greedy_Best_First_Search(state_robot, goal, image, map_resolution)

start_time = time.time()
path = planner.plan(max_iteration,visualize)
print("Planning time: ", time.time()-start_time)

# Interpolate path with splines ---------------------------------------
spline, xs = interpolate_path(path, dt)
path_spline = []
for i in range(int(len(path)/dt)):
    temp = spline(xs[i])
    path_spline.insert(0,temp*map_resolution)
path_spline = np.array(path_spline)


# Control ---------------------------------------
#horizon = 10
#controller = Casadi_nmpc(horizon,[],[], dt)

b = 0.1
k1 = 3
k2 = 3
horizon = 1
controller = IO_linearization(b,k1,k2, dt)

state_evolution = [copy.copy(state_robot)]
for j in range(np.shape(path_spline)[0]):
    state_evolution = np.append(state_evolution, [copy.copy(state_robot)], axis=0)

    start_time = time.time()

    reference_x = []
    reference_y = []
    for i in range(horizon):
        if(j+i < np.shape(path_spline)[0]):
            reference_x.append(path_spline[j+i][0])
            reference_y.append(path_spline[j+i][1])
        else:
            reference_x.append(path_spline[-1][0])
            reference_y.append(path_spline[-1][1])


    v, w = controller.compute_control(state_robot, reference_x, reference_y)
    
    print("Control time: ", time.time()-start_time)
    state_robot = robot.integrate(state_robot, v, w)
    



# Plotting ---------------------------------------
state_evolution = np.array(state_evolution)
plt.figure("Tracking Performance") 
plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

plt.subplot(211)
plt.plot(state_evolution[:,0])
plt.plot(path_spline[:,0])
plt.ylabel('x')
plt.legend(['robot', 'path'])

plt.subplot(212)
plt.plot(state_evolution[:,1])
plt.plot(path_spline[:,1])
plt.ylabel('y')
plt.show()




