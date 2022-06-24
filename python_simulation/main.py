import numpy as np
import matplotlib.pyplot as plt
from pgm_reader import Reader
from planners.A_star import *

f = './maps/map.pgm'
reader = Reader()
image = reader.read_pgm(f)
width = reader.width
height = reader.height

print("width", width)
print("width", height)
print("data", image[10][10])



f = plt.figure()
f.figimage(image)
plt.show()



## initial state of the robot, goal position, hyperparameters
state_robot = np.array([1, 1, 0])  # x y theta
dt = 0.1                           # sampling time
goal = np.array([2.5, 0])          # goal
max_iteration = 1000 #max iterations planner



path = A_star(state_robot, goal, image)