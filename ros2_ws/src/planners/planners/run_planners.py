import itertools
import rclpy # type: ignore
from rclpy.node import Node # type: ignore

from std_msgs.msg import String # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from tf2_msgs.msg import TFMessage # type: ignore
from nav_msgs.msg import OccupancyGrid, Path  # type: ignore
import tf_transformations # type: ignore

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

import sys
import copy
import matplotlib.pyplot as plt # type: ignore
import math
from scipy.interpolate import CubicSpline # type: ignore
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)

import threading

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))


sys.path.append(dir_path + '/../../../../python_scripts/planners/')
from grid_based.a_star import A_star # type: ignore 
from grid_based.greedy_best_first_search import Greedy_Best_First_Search # type: ignore
from grid_based.breadth_first_search import Breadth_First_Search # type: ignore
from grid_based.djikstra import Djikstra # type: ignore

from sampling_based.rrt import RRT # type: ignore
from sampling_based.rrt_primitives import RRT_primitives # type: ignore

sys.path.append(dir_path + '/../../../../python_scripts')
from path_utilities import interpolate_path, filter_map, draw_map # type: ignore

# Collection of multiple planners ---------------------------------------

class Planners(Node):
    def __init__(self):
        super().__init__('Planners')
        # Publishers and Subscribers ---------------------------------------
        self.publisher_path = self.create_publisher(Path, 'path', 1)
        self.publisher_map_filtered = self.create_publisher(OccupancyGrid,'map_filtered',1)

        self.subscription_goal = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        #self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.subscription_map = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.5, self.tf_callback)

        # State robot variable ---------------------------------------
        self.state_robot = np.zeros(3)
        
        # Map Resolution and Max num Iteration ---------------------------------------
        self.map_origin = np.zeros(2)
        self.map = np.zeros((10,10))
        self.map_resolution = 0.05
        self.max_iteration = 1000
        self.visualize = False

        # Plan only if there is a request and a map---------------------------------------
        self.state_arrived = False
        self.map_arrived = False

        # Filtered Map ---------------------------------------
        self.safety = 5
        self.useFilteredMap = True

        # Spline dt ---------------------------------------
        self.dt = 0.1


        # Plan Choice ---------------------------------------
        self.which_planner = 1


        # Interactive Command Line ----------------------------
        t1 = threading.Thread(target=self.interactive_command_line)
        t1.daemon = True
        t1.start()



    # Goal Callback ---------------------------------------
    def goal_callback(self, msg):
        if(self.state_arrived == True and self.map_arrived == True):
            goal = np.array([msg.pose.position.x, msg.pose.position.y])
            goal_shifted = np.zeros(2)
            goal_shifted[0] = goal[0] - self.map_origin[0]
            goal_shifted[1] = goal[1] - self.map_origin[1]

            state_shifted = np.zeros(3)
            state_shifted[0] = self.state_robot[0] - self.map_origin[0]
            state_shifted[1] = self.state_robot[1] - self.map_origin[1]

            print("#########")
            print("Start: ", state_shifted)
            print("Goal: ", goal_shifted)

            state_shifted[0] = round(state_shifted[0],1)
            state_shifted[1] = round(state_shifted[1],1)

            goal_shifted[0] = round(goal_shifted[0],1)
            goal_shifted[1] = round(goal_shifted[1],1)


            # Plan ---------------------------------------
            if(self.which_planner == 1):
                planner = A_star(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))
            
            elif(self.which_planner == 2):
                planner = Greedy_Best_First_Search(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))
            
            elif(self.which_planner == 3):
                planner = Breadth_First_Search(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))
            
            elif(self.which_planner == 4):
                planner = Djikstra(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))
            
            elif(self.which_planner == 5):
                planner = RRT(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))
            
            elif(self.which_planner == 6):
                planner = RRT_primitives(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))

            path = planner.plan(self.max_iteration,self.visualize)
            

            # Spline for smoothing ---------------------------------------
            spline, xs = interpolate_path(path, self.dt)


            # Publish new path ---------------------------------------
            path_msg = Path()
            path_msg.header.frame_id = "map"
            for i in range(int(len(path)/0.1)):
                temp = spline(xs[i])
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = temp[0]*self.map_resolution + self.map_origin[0]
                poseStamped.pose.position.y = temp[1]*self.map_resolution + self.map_origin[1]

                poseStamped.header.frame_id = "map"
                path_msg.poses.append(poseStamped)

            self.publisher_path.publish(path_msg)


    # Robot state callback ---------------------------------------
    def tf_callback(self,):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time())
            quaternion = [float(t.transform.rotation.x), float(t.transform.rotation.y), 
                float(t.transform.rotation.z), float(t.transform.rotation.w)]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            
            self.state_robot[0] = t.transform.translation.x #???
            self.state_robot[1] = t.transform.translation.y
            self.state_robot[2] = euler[2] 
            #print("pose ", self.state_robot)
            self.state_arrived = True
        except TransformException as ex:
            return

    # Map Callback ---------------------------------------
    def map_callback(self, msg):
        self.map = np.zeros((msg.info.height,msg.info.width))
        d = 0
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                self.map[i][j] = 254 if (msg.data[d] < 10 and msg.data[d] >= 0) else 0
                d = d+1

        if(self.useFilteredMap):
            self.map_filtered = filter_map(self.map, self.safety)
            self.publish_map_filter(msg)
            self.map = self.map_filtered


        self.map = np.flip(self.map, 0)
        self.map = np.rot90(self.map, axes=(1, 0))

        self.map_origin[0] = msg.info.origin.position.x
        self.map_origin[1] = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution

        
        self.map_arrived = True


    # Publish safer map ---------------------------------------
    def publish_map_filter(self, msg):
        d = 0
        for i, j in itertools.product(range(msg.info.height), range(msg.info.width)):
            msg.data[d] = 0 if self.map_filtered[i][j] == 254 else 100
            d = d + 1
        self.publisher_map_filtered.publish(msg)



    def interactive_command_line(self, ):
        print("---- You can change the planner by pressing -------")
        print("1: A*")
        print("2: Greedy Best First Search")
        print("3: Breadth First Search")
        print("4: Djikstra")
        print("5: RRT")
        print("6: RRT with primitives")
        print("---------------------------------------------------")
        while True:
            new_planner = input(">>> ")
            if(new_planner == "1"):
                self.which_planner = 1
                print("## Planner started with A* ##")
            elif(new_planner == "2"):
                self.which_planner = 2
                print("## Planner started with Greedy Best First Search ##")
            elif(new_planner == "3"):
                self.which_planner = 3
                print("## Planner started with Breadth First Search ##")
            elif(new_planner == "4"):
                self.which_planner = 4
                print("## Planner started with Djikstra ##")
            elif(new_planner == "5"):
                self.which_planner = 5
                print("## Planner started with RRT ##")
            elif(new_planner == "6"):
                self.which_planner = 6
                print("## Planner started with RRT with primitives ##")
            else:
                print("Wrong input")
                continue

def main(args=None):
    rclpy.init(args=args)
    print("## Planner started with A* ##")

    planner_node = Planners()

    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()