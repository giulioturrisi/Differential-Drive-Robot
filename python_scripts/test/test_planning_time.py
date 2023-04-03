import sys
sys.path.append('/home/python_scripts/planners/')
from grid_based.a_star import A_star # type: ignore 
from grid_based.greedy_best_first_search import Greedy_Best_First_Search # type: ignore
from grid_based.breadth_first_search import Breadth_First_Search # type: ignore
from grid_based.djikstra import Djikstra # type: ignore

from sampling_based.rrt import RRT # type: ignore
from sampling_based.rrt_primitives import RRT_primitives # type: ignore

import time

import numpy as np
import matplotlib.pyplot as plt
from pgm_reader import Reader


if __name__ == "__main__":
    f = '/home/python_scripts/maps/map.pgm'
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


    for k in range(0,6):
        start_time = time.time()
        if(k==0):
            planner = RRT(state_robot, goal, image, resolution)
        elif(k==1):
            planner = RRT_primitives(state_robot, goal, image, resolution)
        elif(k==2):
            planner = A_star(state_robot, goal, image, resolution)
        elif(k==3):
            planner = Djikstra(state_robot, goal, image, resolution)
        elif(k==4):
            planner = Breadth_First_Search(state_robot, goal, image, resolution)
        elif(k==5):
            planner = Greedy_Best_First_Search(state_robot, goal, image, resolution)
        
        path = planner.plan(max_iteration, False)

        if(k==0):
            print("planning time RRT: ", time.time()-start_time)
        elif(k==1):
            print("planning time RRT primitives: ", time.time()-start_time)
        elif(k==2):
            print("planning time A star: ", time.time()-start_time)
        elif(k==3):
            print("planning time Djikstra: ", time.time()-start_time)
        elif(k==4):
            print("planning time Breadth First Search: ", time.time()-start_time)
        elif(k==5):
            print("planning time Greedy Best First Search: ", time.time()-start_time)
        