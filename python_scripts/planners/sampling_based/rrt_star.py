import math

import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore

import os
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path)
from rrt import RRT # type: ignore



class RRT_star(RRT):
    """Path planning with RRT*: like RRT, but every new node is connected to the neighbour that gives
    the cheapest path from the start, and the neighbours are rewired through the new node when this
    shortens their path. After the first solution the tree keeps improving for refinement_iterations
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

        self.goal_bias = 0.1                        # probability of sampling the goal
        self.max_rewire_radius = 2*self.step_size   # [cells]
        self.refinement_iterations = 1000           # iterations after the first solution

        # gamma of the rewiring radius gamma*sqrt(log(n)/n), from the free area of the map (Karaman and Frazzoli)
        free_area = max(1, np.count_nonzero(self.map == 254))
        self.gamma = 2*math.sqrt(1.5)*math.sqrt(free_area/math.pi)

        # cost from the start of each node, children of each node and nodes on the goal
        self.cost = np.zeros(1)
        self.children = [[]]
        self.num_nodes = 1
        self.goal_nodes = []


    def init_tree(self, max_nodes):
        """Allocate the tree, the start is the root
        Args:
            max_nodes (int): maximum number of nodes
        """
        root = self.node_opened[0]
        self.node_opened = np.zeros((max_nodes, 3))
        self.node_opened[0] = [root[0], root[1], -1]
        self.cost = np.zeros(max_nodes)
        self.children = [[]]
        self.num_nodes = 1
        self.goal_nodes = []


    def solution_cost(self,):
        """Cost of the best path found so far
        Returns:
            (float): cost [cells], inf if the goal is not reached yet
        """
        if(len(self.goal_nodes) == 0):
            return math.inf
        return float(np.min(self.cost[self.goal_nodes]))


    def find_nearest_node(self, desired_node):
        """Find the index of the node neared to a desired one
        Args:
            desired_node (list): x-y coordinate of the desired node
        Returns:
            (int): index in the global list of node point
        """
        nodes = self.node_opened[:self.num_nodes]
        distances = np.hypot(nodes[:, 0] - desired_node[0], nodes[:, 1] - desired_node[1])
        return int(np.argmin(distances))


    def rewire_radius(self,):
        """Radius of the neighbourhood of a new node, it shrinks while the tree grows
        Returns:
            (float): radius [cells]
        """
        n = self.num_nodes + 1
        return min(self.max_rewire_radius, max(self.step_size, self.gamma*math.sqrt(math.log(n)/n)))


    def add_node(self, new_node, nearest_node_index):
        """Add a collision free node choosing its best parent, then rewire the neighbours through it
        Args:
            new_node (list): x-y coordinate of the new node
            nearest_node_index (int): nearest node, already connected without collisions
        Returns:
            (int): index of the new node
        """
        x, y = new_node[0], new_node[1]
        nodes = self.node_opened[:self.num_nodes]
        distances = np.hypot(nodes[:, 0] - x, nodes[:, 1] - y)
        near = np.union1d(np.flatnonzero(distances <= self.rewire_radius()), [nearest_node_index])
        free_segment = {nearest_node_index: True}

        # Best parent: the first collision free neighbour in order of cost through it
        parent = nearest_node_index
        for k in near[np.argsort(self.cost[near] + distances[near])]:
            if(k == nearest_node_index):
                break
            free_segment[k] = not self.segment_in_collision(nodes[k][0], nodes[k][1], x, y)
            if(free_segment[k]):
                parent = k
                break

        new_index = self.num_nodes
        self.node_opened[new_index] = [x, y, parent]
        self.cost[new_index] = self.cost[parent] + distances[parent]
        self.children.append([])
        self.children[parent].append(new_index)
        self.num_nodes += 1

        # Rewiring: neighbours that are reached with a lower cost passing through the new node
        for k in near:
            new_cost = self.cost[new_index] + distances[k]
            if(k == parent or new_cost >= self.cost[k] - 1e-9):
                continue
            if(k not in free_segment):
                free_segment[k] = not self.segment_in_collision(x, y, nodes[k][0], nodes[k][1])
            if(not free_segment[k]):
                continue

            old_parent = int(self.node_opened[k][2])
            self.children[old_parent].remove(k)
            self.children[new_index].append(k)
            self.node_opened[k][2] = new_index

            # the whole subtree of k gets cheaper
            delta = self.cost[k] - new_cost
            subtree = [k]
            while(len(subtree) > 0):
                j = subtree.pop()
                self.cost[j] -= delta
                subtree.extend(self.children[j])

        return new_index


    def take_path(self, finish):
        """Take the final path
        Args:
            finish (bool): if the goal is reached or time limit, return a path!
        Returns:
            (list): list of nodes contained in the path
        """
        nodes = self.node_opened[:self.num_nodes]
        if(finish == 1):
            # cheapest node on the goal
            last_index = int(self.goal_nodes[np.argmin(self.cost[self.goal_nodes])])
        else:
            last_index = self.find_nearest_node(self.goal)

        path = []
        for _ in range(self.num_nodes):
            path.append([nodes[last_index][0], nodes[last_index][1]])
            if(nodes[last_index][2] < 0):
                break
            last_index = int(nodes[last_index][2])
        return path


    def plan(self, max_iteration, visualize=False):
        """Main function of the planning procedure
        Args:
            max_iteration (int): time limit for the search
            visualize (bool): boolean for plotting the planning procedure
        Returns:
            (list): list of nodes contained in the path
        """
        finish = 0
        iterator = 0
        iteration_first_solution = None

        self.check_start()
        self.init_tree(max_iteration + 2)

        if(visualize):
            rgb_image = np.stack([self.map]*3, axis=2)/255.
            rgb_image[self.start[0]][self.start[1]] = [1,0,0]
            rgb_image[self.goal[0]][self.goal[1]] = [0,0,1]
            f = plt.figure("Planning")
            ax = f.gca()
            h = ax.imshow(rgb_image)


        while (iterator <= max_iteration):
            if(finish == 1 and iterator - iteration_first_solution >= self.refinement_iterations):
                break
            iterator += 1

            sample_node = self.sample()
            nearest_node_index = self.find_nearest_node(sample_node)
            new_node = self.choose_primitives(nearest_node_index, sample_node)

            #check collision
            if(self.in_collision(new_node, nearest_node_index) == True):
                continue

            new_index = self.add_node(new_node, nearest_node_index)
            if(self.check_goal(new_node)):
                self.goal_nodes.append(new_index)
                if(finish == 0):
                    finish = 1
                    iteration_first_solution = iterator

            if(visualize):
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))] = [0,1,0]
                h.set_data(rgb_image)
                plt.pause(0.01)
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))]  = [1,0,0]

        return self.take_path(finish)
