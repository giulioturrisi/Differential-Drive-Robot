import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore



class Djikstra:
    """Path planning with a simple Djikstra algorithm
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

    
        self.map = map

        # node to expand
        self.frontiers = []
        self.frontiers.append([self.start[0], self.start[1], 0])

        # node opened before
        self.come_from = np.ones((self.height,self.width,2),int)*-1;
        self.come_from[self.start[0]][self.start[1]] = [self.start[0],self.start[1]]


        self.cost_so_far = np.ones((self.height,self.width,1),int);
        self.cost_so_far[self.start[0]][self.start[1]] = 0

        self.graph_cost = 1
        

        self.node_opened = []


    def find_next_cell(self,):
        """Find next cell to expand given the cost
        Returns:
            (list): x and y coordinate of the new point
        """
        best_node = []
        best_cost = 1000
        best_index = -1
        for i in range(len(self.frontiers)):
            temp = self.frontiers[i][:2]
            temp_cost = self.frontiers[i][2]
            if(temp_cost < best_cost):
                best_cost = temp_cost
                best_node = temp
                best_index = i
        
        if(best_index == -1):
            return best_node
        else:
            self.frontiers.pop(best_index)
            self.node_opened.append([best_node[0],best_node[1]])
            return best_node


    def find_nearest_cell(self,desired_cell):
        """Find the nearest cell given a desired cell
        Args:
            desired_node (list): x-y coordinate of the desired node
        Returns:
            (int): index in the global list of node point
        """
        best_node = []
        best_cost = 1000
        for i in range(len(self.node_opened)):
            temp = self.node_opened[i]
            #h = abs(self.goal[0]-(temp[0])) + abs(self.goal[1]-(temp[1]))
            h = np.sqrt(np.power(desired_cell[0]-(temp[0]),2) + np.power(desired_cell[1]-(temp[1]),2))
            if(h < best_cost):
                best_cost = h
                best_node = temp
        return best_node


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


    def open_sons_cell(self ,parent):
        """Expand nodes that are sons of a given parent
        Args:
            parent (list): cell to expand
        """
        x_parent = parent[0];
        y_parent = parent[1];

        #need to check 8 sons!
        for i in range(-1,2):
            for j in range(-1,2):   
                if(i == 0 and j == 0):
                    continue;
                x_new = x_parent + i
                y_new = y_parent + j

                if (x_new < self.height and y_new < self.width):
                    if(self.map[x_new][y_new] != 254):
                        break
                    new_cost = self.cost_so_far[x_parent][y_parent] + self.graph_cost

                    if(np.array_equal(self.come_from[x_new][y_new],np.array([-1,-1])) or new_cost < self.cost_so_far[x_new][y_new]):
                        self.frontiers.append([x_new, y_new, new_cost])     
                        self.come_from[x_new][y_new] = [x_parent,y_parent]  # where i come from!      
                        self.cost_so_far[x_new][y_new] = new_cost
                        

    def check_goal(self,next_cell):
        """Check if the goal is reached
        Args:
            next_cell (list): x-y coordinate of the node to check
        Returns:
            (bool): boolean for goal reaching
        """
        x = next_cell[0]
        y = next_cell[1]
        return 1 if (x == self.goal[0] and y == self.goal[1]) else 0


    def check_start(self):
        """Check if the starting node is feasible - it calls find_new_start()
        """
        if(self.map[self.start[0]][self.start[1]] != 254):
            print("Start is unfeasible!")
            new_start = self.find_new_start()
            print("New start found..")
            self.start = new_start
            self.frontiers = []
            self.frontiers.append([self.start[0], self.start[1], 0])

            # node opened before
            self.come_from = np.ones((self.height,self.width,2),int)*-1;
            self.come_from[self.start[0]][self.start[1]] = [self.start[0],self.start[1]]


            self.cost_so_far = np.ones((self.height,self.width,1),int)*1000;
            self.cost_so_far[self.start[0]][self.start[1]] = 0
        return


    def take_path(self, finish):
        """Take the final path
        Args:
            finish (bool): if the goal is reached or time limit, return a path!
        Returns:
            (list): list of nodes contained in the path
        """
        if(finish == 1):
            last_cell = [self.goal[0],self.goal[1]];
        else:
            last_cell = self.find_nearest_cell(self.goal)

        path = [last_cell]
        last_cell = self.come_from[last_cell[0]][last_cell[1]]


        for _ in range(self.width*self.width):
            path.append(last_cell.tolist()) 
            if(last_cell[0] == self.start[0] and last_cell[1] == self.start[1]):
                break
            last_cell = self.come_from[last_cell[0]][last_cell[1]]

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
            next_cell = self.find_next_cell()
            if(next_cell == []):
                break
            
            finish = self.check_goal(next_cell)
            if(finish == 1):
                break
            self.open_sons_cell(next_cell)

            iterator += 1

            if(visualize):
                rgb_image[next_cell[0]][next_cell[1]] = [0,1,0]
                h.set_data(rgb_image)
                plt.pause(0.01)
                rgb_image[next_cell[0]][next_cell[1]] = [1,0,0]


        return self.take_path(finish)


