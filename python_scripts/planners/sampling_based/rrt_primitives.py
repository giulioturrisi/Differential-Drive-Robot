import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore
import math


class RRT_primitives:
    """Path planning with a simple RRT with a primitives interconnection between nodes
    """
    def __init__(self, start, goal, map, resolution):
        """Init func
        Args:
            start (np.array): actual state of the robot
            goal (np.array): desired goal 
            map (np.array): multidimensional array containing the map
            resolution (float): dimension of each cell
        """
        self.resolution = resolution
        self.start = np.array([int(start[0]/resolution), int(start[1]/resolution)])
        self.goal = np.array([int(goal[0]/resolution), int(goal[1]/resolution)])


        map_shape = map.shape 
        self.height = map_shape[0]  
        self.width = map_shape[1] 

        
        self.multipling_factor = 5
        self.dt = 1
        self.goal_bias = 0.7

        self.map = map

        #x, y, theta, control inputs, parent_node,
        self.node_opened = np.array([[self.start[0], self.start[1], start[2], 0, 0, -1]])

        #primitives
        forward = [1,0]
        backward = [-1,0]
        turn_left = [0, -1]
        turn_right = [0, 1]
        arc_left_forward = [1, -1]
        arc_right_forward = [1, 1]
        arc_left_backward = [-1, -1];
        arc_right_backward = [-1, 1];

        self.primitives = [forward, backward , turn_left, turn_right, arc_left_forward, arc_right_forward, arc_left_backward, arc_right_backward]
    

    def sample(self,):
        """Sample a new random point
        Returns:
            (list): x and y coordinate of the new point
        """
        rand = np.random.rand()
        if(rand > self.goal_bias):
            desired_node = [self.goal[0], self.goal[1]]
        else:
            rand_x = np.random.rand()*self.height;
            rand_y = np.random.rand()*self.width;
            desired_node = [rand_x, rand_y];
        return desired_node


    def find_nearest_node(self, desired_node):
        """Find the index of the node neared to a desired one
        Args:
            desired_node (list): x-y coordinate of the desired node
        Returns:
            (int): index in the global list of a node point
        """
        best_node = []
        best_cost = 1000
        best_index = 0
        for i in range(len(self.node_opened)):
            temp = self.node_opened[i]

            h = np.sqrt(np.power(desired_node[0]-(temp[0]),2) + np.power(desired_node[1]-(temp[1]),2))
            if(h < best_cost):
                best_cost = h
                best_node = temp
                best_index = i
        return best_index


    def sample_primitives(self,rand_index):
        """Connect two node with a random sampled primitives
        Args:
            near_index (int): index of the nearest node
            desired_node (list): x-y coordinate of the desired node
        Returns:
            (list): a new node is generated connecting the near node and the desired node with the line
            (bool): boolean for reporting collision using the primitive
        """
        #rand_index = np.random.randint(self.node_opened.shape[0])
        near_node = self.node_opened[rand_index]
        
        len_primitives = len(self.primitives)
        rand_primitive_index = np.random.randint(len_primitives)
        primitive = self.primitives[rand_primitive_index]

        v = primitive[0]*self.multipling_factor
        w = primitive[1]*self.multipling_factor


        x_new1 = near_node[0] + v*math.cos(near_node[2])*self.dt/3.
        y_new1 = near_node[1] + v*math.sin(near_node[2])*self.dt/3.
        theta_new1 = near_node[2] + w*self.dt/3.
        collision_1 = self.in_collision([x_new1, y_new1, theta_new1])

        x_new2 = x_new1 + v*math.cos(theta_new1)*self.dt/3.
        y_new2 = y_new1 + v*math.sin(theta_new1)*self.dt/3.
        theta_new2 = theta_new1 + w*self.dt/3.
        collision_2 = self.in_collision([x_new2, y_new2, theta_new2])

        
        x_new3 = x_new2 + v*math.cos(theta_new2)*self.dt/3.
        y_new3 = y_new2 + v*math.sin(theta_new2)*self.dt/3.
        theta_new3 = theta_new2 + w*self.dt/3.
        collision_3 = self.in_collision([x_new3, y_new3, theta_new3])     
        
        collision = collision_1 or collision_2 or collision_3


        new_node = [x_new3, y_new3, theta_new3, v, w, rand_index]

        return new_node, collision



    def find_new_start(self,):
        """If the start node is unfeasible, choose a new start near the initial one
        Returns:
            (list): a new feasible starting goal
        """
        best_node = []
        best_cost = 1000
        for i in range(self.height):
            for j in range(self.width):
                temp = self.map[i][j]
                h = np.sqrt(np.power(self.start[0]-(i),2) + np.power(self.start[1]-(j),2))
                if(h < best_cost and self.map[i][j] == 254):
                    best_cost = h
                    best_node = [i, j]
        return best_node


    def in_collision(self, node_to_check):
        """Check multiple point collisions while connecting two nodes
        Args:
            node_to_check (list): x-y coordinate of the node to check
            nearest_node_index (int): starting node of the primitive
        Returns:
            (bool): boolean for collision checking
        """
        x = node_to_check[0]
        y = node_to_check[1]

        if(self.map[int(round(x))][int(round(y))] != 254):
            return True

        return False
                        

    def check_goal(self, next_cell):
        """Check if the goal is reached
        Args:
            next_cell (list): x-y coordinate of the node to check
        Returns:
            (bool): boolean for goal reaching
        """
        x = next_cell[0]
        y = next_cell[1]

        h = np.sqrt(np.power(self.goal[0]-x,2) + np.power(self.goal[1]-y,2))
        if (h < 0.1):
            return True
        else:
            return False


    def check_start(self):
        """Check if the starting node is feasible - it calls find_new_start()
        """
        if(self.map[self.start[0]][self.start[1]] != 254):
            print("Start is unfeasible!")
            new_start = self.find_new_start()
            print("New start found..")
            self.start = new_start
            self.node_opened = []
            self.node_opened.append([self.start[0], self.start[1], 0])

        return


    def take_path(self, finish):
        """Take the final path
        Args:
            finish (bool): if the goal is reached or time limit, return a path!
        Returns:
            (list): list of nodes contained in the path
        """
        if(finish == 1):
            last_node = self.node_opened[-1]
        else:
            last_index = self.find_nearest_node(self.goal)
            last_node = self.node_opened[int(last_index)]

        path = [[last_node[0], last_node[1]]]
        last_node = self.node_opened[int(last_node[-1])]

        for _ in range(self.width*self.width):
            temp = last_node.tolist()
            path.append([temp[0], temp[1]])
            if(last_node[0] == self.start[0] and last_node[1] == self.start[1]):
                break
            last_node = self.node_opened[int(last_node[-1])]
        
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

        self.check_start()

        if(visualize):
            rgb_image = np.stack([self.map]*3, axis=2)/255.
            rgb_image[self.start[0]][self.start[1]] = [1,0,0]
            rgb_image[self.goal[0]][self.goal[1]] = [0,0,1]
            f = plt.figure("Planning")   
            ax = f.gca() 
            h = ax.imshow(rgb_image)


        while (finish == 0 and iterator <= max_iteration):
            sample_node = self.sample()
            nearest_node_index = self.find_nearest_node(sample_node)
            new_node, collision = self.sample_primitives(nearest_node_index)
            
            #check collision
            if(collision == True):
                continue

            self.node_opened = np.concatenate((self.node_opened, [new_node]), axis = 0)
            finish = self.check_goal(new_node)
            if(finish == 1):
                print("finish!")
                break


            iterator += 1

            if(visualize):
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))] = [0,1,0]
                h.set_data(rgb_image)
                plt.pause(0.01)
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))]  = [1,0,0]


        return self.take_path(finish)

   




