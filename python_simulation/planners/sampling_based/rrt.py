import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore



class RRT:
    def __init__(self, start, goal, map, resolution):
        self.resolution = resolution
        self.start = np.array([int(start[0]/resolution), int(start[1]/resolution)])
        self.goal = np.array([int(goal[0]/resolution), int(goal[1]/resolution)])


        map_shape = map.shape 
        self.height = map_shape[0]  
        self.width = map_shape[1] 

        
        self.goal_bias = 0.5

        self.map = map

        #x, y, theta, parent_node, v, w
        self.node_opened = [self.start[0], self.start[1], 0, 0]

    
    def sample(self,goal_bias):
        rand = np.random.rand()
        if(rand > self.goal_bias):
            desired_node = [self.goal[0], self.goal[1], 0]
        else:
            rand_x = np.random.rand()*self.height;
            rand_y = np.random.rand()*self.width;
            rand_z = 0;
            desired_node = [rand_x, rand_y, rand_z];
        return desired_node

    def choose_primitives(self, near_index, desired_node):

        near_node = self.nodes[near_index]
        
        best_index = 1;
        best_distance = 10000;
        best_node = [0, 0, 0];

        
        u1 = desired_node[0] - near_node[0];
        u1 = u1*0.2

        u2 = desired_node[1] - near_node[1];
        u2 = u2*0.2

        control = [u1, u2];

        x_new = near_node[0] + u1;
        y_new = near_node[1] + u2;
        theta_new = 0;
        
        best_node = [x_new, y_new, theta_new]
        new_node = [best_node, near_index, control]

        return new_node



    def find_nearest_cell(self,desired_cell):
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


    def check_collision(self, node_to_check):
        x = node_to_check[0];
        y = node_to_check[1];

        #to check collision!
        return False
                        

    def check_goal(self,next_cell):
        x = next_cell[0]
        y = next_cell[1]

        h = np.sqrt(np.power(self.start[0]-x,2) + np.power(self.start[1]-y,2))
        if (h < 0.1):
            return True
        else:
            return False

    def check_start(self):
        if(self.map[self.start[0]][self.start[1]] != 254):
            print("Start is unfeasible!")
            new_start = self.find_new_start()
            print("New start found..")
            self.start = new_start
            self.node_opened = []
            self.node_opened.append([self.start[0], self.start[1], 0])

        return


    def take_path(self, finish):
        if(finish == 1):
            last_cell = [self.goal[0],self.goal[1]];
        else:
            last_cell = self.find_nearest_cell(self.goal)

        path = [last_cell]
        last_cell = self.node_opened[last_cell[0]][last_cell[1]]



        for _ in range(self.width*self.width):
            path.append(last_cell.tolist())
            if(last_cell[0] == self.start[0] and last_cell[1] == self.start[1]):
                break
            last_cell = self.come_from[last_cell[0]][last_cell[1]]
        #print("path",path)
        return path

   




    def plan(self, max_iteration, visualize=False):
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

   




