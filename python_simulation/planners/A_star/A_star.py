import numpy as np

class A_star:
    def __init__(self, start,goal,map,resolution):
        self.resolution = resolution
        self.start = np.array([int(start[0]/resolution), int(start[1]/resolution)])
        self.goal = np.array([int(goal[0]/resolution), int(goal[1]/resolution)])


        
        map_shape = map.shape # 377,381
        self.height = map_shape[0] #size_y - 377
        self.width = map_shape[1] #size_x  - 381

        
        #to check here is wrong, should be self.width*self.height
        self.map = np.zeros((self.width*self.width,6))
        self.cells_isopen = np.zeros((self.width*self.width,3));


        for i in range(0, self.height):
            for j in range(0, self.width): 
                if(map[i][j] < 250): 
                    self.cells_isopen[i + j*self.width][:] = [-1,10000,10000];
                else:
                    h = 0;
                    self.map[i+j*self.width][:] = [i,j,0,h,0,0];
                    #print("index",i*self.height +j)
                    
                    if(i == self.start[0] and j == self.start[1]):
                        #print("start at", i*self.width + j)
                        self.cells_isopen[i + j*self.width][:] = [1,0,0];
                    else:
                        self.cells_isopen[i + j*self.width][:] = [0,10000,10000];


    def find_next_cell(self,):
        best_cost = 10000
        best_cell = -1
        for i in range(0, self.height):
            for j in range(1, self.width):
                index = i+j*self.width;
                if(self.cells_isopen[index][0] == 1 and self.cells_isopen[index][1] <= best_cost):
                    best_cost = self.cells_isopen[index][1]
                    best_cell = index
        return best_cell


    def check_goal(self,index):
        x = self.map[index][0];
        y = self.map[index][1];
        if(x == self.goal[0] and y == self.goal[1]):
            finish = 1;
        else:
            finish = 0;
        return finish


    def take_path(self,):
        index = int(self.goal[0] + (self.goal[1])*self.width);
        print("index",index)
        path = np.ones((100,6));
        dimension_path = 0;
        for i in range(0,100):
            path[i][0:2] = self.map[index][0:2]*self.resolution
            index = int(self.map[index][5]);
            dimension_path = dimension_path + 1
            if(index == 0):
                break
        path = path[0:dimension_path][:]
        return path




    def open_sons_cell(self ,parent_index):
        x = self.map[parent_index][0];
        y = self.map[parent_index][1];

        cost_parent = self.cells_isopen[parent_index][2];
        self.cells_isopen[parent_index][0] = -1;
        
        #need to check 8 sons!
        for i in range(-1,2):
            for j in range(-1,2):   
                if(i == 0 and j == 0):
                    continue;
                x_new = x + i
                y_new = y + j
                if(x_new<= self.width and y_new <= self.height):
                    
                    #calculate cost 
                    graph_cost = 1               
                    new_cost = cost_parent+graph_cost

                    son_index = int(x_new+(y_new-1)*self.width)

                    
                    if(self.cells_isopen[son_index][0] != -1 and self.cells_isopen[son_index][0] == 0):
                        #print("different from -1")
                        if (new_cost < self.cells_isopen[son_index][1]):
                            h = abs(self.goal[0]-(x_new)) + abs(self.goal[1]-(y_new))

                            
                            self.cells_isopen[son_index][2] = new_cost
                            new_cost = new_cost + h
                            
                            #new_cost
                            self.cells_isopen[son_index][1] = new_cost
                            self.map[son_index][5] = int(parent_index)
                            self.cells_isopen[son_index][0] = 1



