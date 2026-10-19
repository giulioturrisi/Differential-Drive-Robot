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


        self.multipling_factor = 5          # length of a primitive [cells]
        self.turning_angle = math.pi/4.     # heading change of a turning primitive [rad]
        self.dt = 1
        self.goal_bias = 0.3                # probability of sampling the goal
        self.goal_tolerance = self.multipling_factor      # the last straight connection is at most one primitive long [cells]
        self.collision_step = 0.5           # distance between two collision checks along a primitive [cells]

        self.map = map

        #x, y, theta, control inputs, parent_node,
        self.node_opened = np.array([[self.start[0], self.start[1], start[2], 0, 0, -1]], dtype=float)

        #primitives [v, w], w > 0 turns counterclockwise
        forward = [1,0]
        backward = [-1,0]
        turn_left = [0, 1]
        turn_right = [0, -1]
        arc_left_forward = [1, 1]
        arc_right_forward = [1, -1]
        arc_left_backward = [-1, 1];
        arc_right_backward = [-1, -1];

        self.primitives = [forward, backward , turn_left, turn_right, arc_left_forward, arc_right_forward, arc_left_backward, arc_right_backward]


    def sample(self,):
        """Sample a new random point
        Returns:
            (list): x and y coordinate of the new point
        """
        rand = np.random.rand()
        if(rand < self.goal_bias):
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
        distances = np.hypot(self.node_opened[:, 0] - desired_node[0], self.node_opened[:, 1] - desired_node[1])
        return int(np.argmin(distances))


    def simulate_primitive(self, node, primitive):
        """Integrate a primitive starting from a node, checking collisions along the motion
        Args:
            node (np.array): starting node
            primitive (list): normalized linear and angular speed of the primitive
        Returns:
            (list): final x, y, theta and control inputs (None if there is a collision)
        """
        v = primitive[0]*self.multipling_factor
        w = primitive[1]*self.turning_angle

        num_steps = max(1, int(math.ceil(abs(v)*self.dt/self.collision_step)))
        x = node[0]
        y = node[1]
        theta = node[2]
        for _ in range(num_steps):
            x_previous, y_previous = x, y
            x = x + v*math.cos(theta)*self.dt/num_steps
            y = y + v*math.sin(theta)*self.dt/num_steps
            theta = theta + w*self.dt/num_steps
            if(self.segment_in_collision([x_previous, y_previous], [x, y])):
                return None

        return [x, y, theta, v, w]


    def sample_primitives(self, near_index, desired_node):
        """Connect two node with the primitive that gets closer to the desired node
        Args:
            near_index (int): index of the nearest node
            desired_node (list): x-y coordinate of the desired node
        Returns:
            (list): a new node is generated connecting the near node and the desired node with a primitive
            (bool): boolean for reporting collision of all the primitives
        """
        near_node = self.node_opened[near_index]

        best_node = None
        best_distance = np.inf
        for primitive in self.primitives:
            new_node = self.simulate_primitive(near_node, primitive)
            if(new_node is None):
                continue
            distance = math.hypot(new_node[0] - desired_node[0], new_node[1] - desired_node[1])
            if(distance < best_distance):
                best_distance = distance
                best_node = new_node + [near_index]

        return best_node, best_node is None



    def find_new_start(self,):
        """If the start node is unfeasible, choose a new start near the initial one
        Returns:
            (list): a new feasible starting goal
        """
        best_node = []
        best_cost = np.inf
        for i in range(self.height):
            for j in range(self.width):
                temp = self.map[i][j]
                h = np.sqrt(np.power(self.start[0]-(i),2) + np.power(self.start[1]-(j),2))
                if(h < best_cost and self.map[i][j] == 254):
                    best_cost = h
                    best_node = [i, j]
        return best_node


    def cell_occupied(self, i, j):
        """Check if a cell is not free (outside the map is not free)
        Args:
            i (int): x index of the cell
            j (int): y index of the cell
        Returns:
            (bool): boolean for occupied cell
        """
        if(i < 0 or j < 0 or i >= self.height or j >= self.width):
            return True
        return self.map[i][j] != 254


    def in_collision(self, node_to_check):
        """Check if a node is in collision (outside the map is a collision)
        Args:
            node_to_check (list): x-y coordinate of the node to check
        Returns:
            (bool): boolean for collision checking
        """
        return self.cell_occupied(int(round(node_to_check[0])), int(round(node_to_check[1])))


    def segment_in_collision(self, node_from, node_to):
        """Check every cell crossed by the straight segment between two nodes (grid traversal of
        Amanatides and Woo), the cell (i, j) contains the points rounding to (i, j)
        Args:
            node_from (list): x-y coordinate of the first node
            node_to (list): x-y coordinate of the second node
        Returns:
            (bool): boolean for collision checking
        """
        x_start, y_start = node_from[0] + 0.5, node_from[1] + 0.5
        x_end, y_end = node_to[0] + 0.5, node_to[1] + 0.5
        i, j = math.floor(x_start), math.floor(y_start)
        i_end, j_end = math.floor(x_end), math.floor(y_end)
        dx, dy = x_end - x_start, y_end - y_start

        step_i = 1 if dx > 0 else -1
        step_j = 1 if dy > 0 else -1
        t_max_x = ((i + (step_i > 0)) - x_start)/dx if dx != 0 else math.inf
        t_max_y = ((j + (step_j > 0)) - y_start)/dy if dy != 0 else math.inf
        t_delta_x = abs(1/dx) if dx != 0 else math.inf
        t_delta_y = abs(1/dy) if dy != 0 else math.inf

        for _ in range(abs(i_end - i) + abs(j_end - j) + 1):
            if(self.cell_occupied(i, j)):
                return True
            if(t_max_x < t_max_y):
                t_max_x += t_delta_x
                i += step_i
            else:
                t_max_y += t_delta_y
                j += step_j
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
        if (h < self.goal_tolerance):
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
            self.start = np.array(new_start)
            start_theta = self.node_opened[0][2]
            self.node_opened = np.array([[self.start[0], self.start[1], start_theta, 0, 0, -1]], dtype=float)

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

        for _ in range(self.height*self.width):
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
            # every iteration counts, also the ones that end in a collision
            iterator += 1

            sample_node = self.sample()
            nearest_node_index = self.find_nearest_node(sample_node)
            new_node, collision = self.sample_primitives(nearest_node_index, sample_node)

            #check collision
            if(collision == True):
                continue

            self.node_opened = np.concatenate((self.node_opened, [new_node]), axis = 0)

            # close to the goal: connect it with a straight segment if free
            if(self.check_goal(new_node) and not self.segment_in_collision(new_node, self.goal)):
                goal_node = [self.goal[0], self.goal[1], new_node[2], 0, 0, len(self.node_opened) - 1]
                self.node_opened = np.concatenate((self.node_opened, [goal_node]), axis = 0)
                finish = 1
                break

            if(visualize):
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))] = [0,1,0]
                h.set_data(rgb_image)
                plt.pause(0.01)
                rgb_image[int(round(new_node[0]))][int(round(new_node[1]))]  = [1,0,0]


        return self.take_path(finish)





