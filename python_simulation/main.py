
import sys
sys.path.append('/home/python_simulation/planners/A_star')
from A_star import A_star
sys.path.append('/home/python_simulation/planners/')
from path_utilities import interpolate_path, filter_map, draw_map

sys.path.append('/home/python_simulation/controllers/casadi_nmpc')
from casadi_nmpc import Casadi_nmpc

from robot_model import Robot

import numpy as np
np.set_printoptions(threshold=sys.maxsize)
import matplotlib.pyplot as plt
from pgm_reader import Reader

import time
import copy

f = './maps/map.pgm'
reader = Reader()
image = reader.read_pgm(f)
width = reader.width
height = reader.height



## initial state of the robot, goal position, hyperparameters
state_robot = np.array([10, 2.45, 0])  # x y theta
dt = 0.1                           # sampling time
goal = np.array([5, 5])          # goal
max_iteration = 1000 #max iterations planner
resolution = 0.05
visualize = False
map_resolution = 0.05

#draw_map(image, state_robot, goal, resolution)


#PLAN
planner = A_star(state_robot, goal, image, map_resolution)
path = planner.plan(max_iteration,visualize)

#print("path",path)
#SPLINE
spline, xs = interpolate_path(path, dt)

path_spline = []
for i in range(int(len(path)/0.1)):
    temp = spline(xs[i])
    path_spline.insert(0,temp*map_resolution)
    #reference_y.append(temp[1])
    #reference_x.append(1)
    #reference_y.append(1)
path_spline = np.array(path_spline)
print("path",path_spline)



horizon = 10
dt = 0.02

controller = Casadi_nmpc(horizon,[],[], dt)
robot = Robot(dt)
controller.initialize_casadi()
state_evolution = []
for j in range(np.shape(path_spline)[0]):
#for j in range(50):
    print("###############")
    print("state robot", state_robot)
    state_evolution.append(copy.copy(state_robot))

    start_time = time.time()

    reference_x = []
    reference_y = []
    for i in range(horizon):
        if(j+i < np.shape(path_spline)[0]):
            reference_x.append(path_spline[j+i][0])
            reference_y.append(path_spline[j+i][1])
            #reference_x.append(0.5)
            #reference_y.append(0.5)
        else:
            reference_x.append(path_spline[-1][0])
            reference_y.append(path_spline[-1][1])
            #reference_x.append(0.5)
            #reference_y.append(0.5)
    controller.initialize_casadi()
    print("reference_x", reference_x)
    v, w = controller.compute_mpc(state_robot, reference_x, reference_y)
    print("control time 2", time.time()-start_time)

    state_robot = robot.integrate(state_robot, v, w)
    

state_evolution = np.array(state_evolution)

print("state evolution", state_evolution)
print("path_spline[:][0]", path_spline[:][0])

plt.plot(state_evolution[:,0])
plt.plot(path_spline[:,0])
plt.show()

plt.plot(state_evolution[:,1])
plt.plot(path_spline[:,1])
plt.show()


