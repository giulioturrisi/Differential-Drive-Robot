from A_star import A_star
import numpy as np
import matplotlib.pyplot as plt
from pgm_reader import Reader

def planning_A_star(start,goal,map, resolution):
    grid_search = A_star(start, goal, map, resolution)
    finish = 0
    while (finish == 0):
        next_cell = grid_search.find_next_cell() 
        if(next_cell == -1):
            finish = 1
            break
        finish = grid_search.check_goal(next_cell)
        if(finish == 1):
            break
        grid_search.open_sons_cell(next_cell)
        
    return grid_search.take_path()



if __name__ == "__main__":
    f = './../../maps/map.pgm'
    reader = Reader()
    image = reader.read_pgm(f)
    width = reader.width
    height = reader.height
    print("width",width)
    print("height",height)
    
    ## initial state of the robot, goal position, hyperparameters
    state_robot = np.array([9, 3, 0])  # x y theta
    dt = 0.1                           # sampling time
    goal = np.array([10, 2.5])          # goal
    max_iteration = 1000 #max iterations planner
    resolution = 0.05



    path = planning_A_star(state_robot, goal, image, resolution)

    print("path",path)