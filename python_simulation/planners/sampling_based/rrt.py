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

        #x, y, theta, parent_node,
        self.node_opened = np.array([[self.start[0], self.start[1], -1]])
    
    def sample(self,):
        rand = np.random.rand()
        if(rand > self.goal_bias):
            desired_node = [self.goal[0], self.goal[1]]
        else:
            rand_x = np.random.rand()*self.height;
            rand_y = np.random.rand()*self.width;
            desired_node = [rand_x, rand_y];
        return desired_node

    def find_nearest_node(self, desired_node):
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


    def choose_primitives(self, near_index, desired_node):

        near_node = self.node_opened[near_index]
        
        best_index = 1;
        best_distance = 10000;
        best_node = [0, 0, 0];

        
        u1 = desired_node[0] - near_node[0];
        u1 = u1*0.1

        u2 = desired_node[1] - near_node[1];
        u2 = u2*0.1

        x_new = near_node[0] + u1;
        y_new = near_node[1] + u2;


        new_node = [x_new, y_new, near_index]

        return new_node




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


    def in_collision(self, node_to_check, nearest_node_index):
        x = node_to_check[0]
        y = node_to_check[1]

        if(self.map[int(round(x))][int(round(y))] != 254):
            return True

        near_node = self.node_opened[nearest_node_index]
        distance_x = x - near_node[0]
        distance_y = y - near_node[1]
        intermidiate_node = [near_node[0] + distance_x/2. , near_node[1] + distance_y/2.];
        if(self.map[int(round(intermidiate_node[0]))][int(round(intermidiate_node[1]))] != 254):
            return True
        

        return False
                        

    def check_goal(self, next_cell):
        x = next_cell[0]
        y = next_cell[1]

        h = np.sqrt(np.power(self.goal[0]-x,2) + np.power(self.goal[1]-y,2))
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
            last_node = self.node_opened[-1]
        else:
            last_index = self.find_nearest_node(self.goal)
            last_node = self.node_opened[int(last_index)]

        path = [[last_node[0], last_node[1]]]
        last_node = self.node_opened[int(last_node[2])]



        for _ in range(self.width*self.width):
            temp = last_node.tolist()
            path.append([temp[0], temp[1]])
            if(last_node[0] == self.start[0] and last_node[1] == self.start[1]):
                break
            last_node = self.node_opened[int(last_node[2])]
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
            sample_node = self.sample()
            nearest_node_index = self.find_nearest_node(sample_node)
            new_node = self.choose_primitives(nearest_node_index, sample_node)
            
            #check collision
            if(self.in_collision(new_node, nearest_node_index) == True):
                continue

            self.node_opened = np.concatenate((self.node_opened, [new_node]), axis = 0)
            finish = self.check_goal(new_node)
            if(finish == 1):
                break
            #self.open_sons_cell(next_cell)

            iterator += 1

            if(visualize):
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))] = [0,1,0]
                h.set_data(rgb_image)
                plt.pause(0.01)
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))]  = [1,0,0]

        #return 0
        return self.take_path(finish)

   




