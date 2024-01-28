import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/../planners/')
from grid_based.a_star import A_star # type: ignore 
from grid_based.greedy_best_first_search import Greedy_Best_First_Search # type: ignore
from grid_based.breadth_first_search import Breadth_First_Search # type: ignore
from grid_based.djikstra import Djikstra # type: ignore

from sampling_based.rrt import RRT # type: ignore
from sampling_based.rrt_primitives import RRT_primitives # type: ignore


import numpy as np
import matplotlib.pyplot as plt
from pgm_reader import Reader


if __name__ == "__main__":
    f = dir_path + '/../maps/map.pgm'
    reader = Reader()
    image = reader.read_pgm(f)
    width = reader.width
    height = reader.height
    print("width",width)
    print("height",height)
    
    ## initial state of the robot, goal position, hyperparameters
    state_robot = np.array([10, 2.45, 0])  # x y theta
    dt = 0.1                           # sampling time
    goal = np.array([5, 2.5])          # goal
    max_iteration = 1000 #max iterations planner
    resolution = 0.05


    print("state_robot",state_robot)
    print("goal",goal)
    print("image", np.shape(image))



    grid_search = RRT(state_robot, goal, image, resolution)
    path = grid_search.plan(max_iteration, True)


    print("path",path)