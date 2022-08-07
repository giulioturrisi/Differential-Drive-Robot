import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore



class Breadth_First_Search:
    def __init__(self, start, goal, map, resolution):
        self.resolution = resolution
        self.start = np.array([int(start[0]/resolution), int(start[1]/resolution)])
        self.goal = np.array([int(goal[0]/resolution), int(goal[1]/resolution)])

        
        map_shape = map.shape 
        self.height = map_shape[0]  
        self.width = map_shape[1] 
 

        self.map = map

        # node to expand
        self.frontiers = []
        self.frontiers.append([self.start[0],self.start[1]])

        # node opened before
        self.come_from = np.ones((self.height,self.width,2),int)*-1;
        self.come_from[self.start[0]][self.start[1]] = [self.start[0],self.start[1]]
        
        self.node_opened = []


    def find_next_cell(self,):
        temp = self.frontiers.pop(0)
        self.node_opened.append([temp[0],temp[1]])
        return temp

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


    def open_sons_cell(self ,parent):
        x_parent = parent[0];
        y_parent = parent[1];




        #need to check 8 sons!
        for i in range(-1,2):
            for j in range(-1,2):   
                if(i == 0 and j == 0):
                    continue;
                x_new = x_parent + i
                y_new = y_parent + j

                if(x_new < self.height and y_new < self.width):
                    
                    
                    if(self.map[x_new][y_new] == 254  and np.array_equal(self.come_from[x_new][y_new],np.array([-1,-1]))):
                        self.frontiers.append([x_new,y_new])     
                        self.come_from[x_new][y_new] = [x_parent,y_parent]  # where i come from!       




    def check_goal(self,next_cell):
        x = next_cell[0];
        y = next_cell[1];
        return 1 if (x == self.goal[0] and y == self.goal[1]) else 0


    def check_start(self):
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


        print("path",path)
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

   



