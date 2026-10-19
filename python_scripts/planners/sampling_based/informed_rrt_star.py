import math

import numpy as np # type: ignore

import os
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path)
from rrt_star import RRT_star # type: ignore



class Informed_RRT_star(RRT_star):
    """Path planning with Informed RRT* (Gammell et al.): like RRT*, but once a path of cost c is found
    the new samples are taken only inside the ellipse with foci in start and goal and major axis c,
    the only region where a node can shorten the path
    """
    def __init__(self, start, goal, map, resolution):
        """Init func
        Args:
            start (np.array): actual state of the robot
            goal (np.array): desired goal
            map (np.array): multidimensional array containing the map
            resolution (float): dimension of each cell
        """
        super().__init__(start, goal, map, resolution)


    def sample(self,):
        """Sample a new random point: in all the map until the goal is reached, then in the informed ellipse
        Returns:
            (list): x and y coordinate of the new point
        """
        c_best = self.solution_cost()
        if(math.isinf(c_best)):
            return super().sample()

        c_min = math.hypot(self.goal[0] - self.start[0], self.goal[1] - self.start[1])
        a = c_best/2.
        b = math.sqrt(max(c_best*c_best - c_min*c_min, 0.0))/2.
        angle = math.atan2(self.goal[1] - self.start[1], self.goal[0] - self.start[0])

        # uniform point in the unit disk, stretched to the ellipse and moved between start and goal
        r = math.sqrt(np.random.rand())
        phi = 2*math.pi*np.random.rand()
        x_ellipse = a*r*math.cos(phi)
        y_ellipse = b*r*math.sin(phi)

        rand_x = (self.start[0] + self.goal[0])/2. + math.cos(angle)*x_ellipse - math.sin(angle)*y_ellipse
        rand_y = (self.start[1] + self.goal[1])/2. + math.sin(angle)*x_ellipse + math.cos(angle)*y_ellipse
        return [rand_x, rand_y]
