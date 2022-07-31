import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid, Path

import tf_transformations

import copy
import matplotlib.pyplot as plt
import math
from scipy.interpolate import CubicSpline


import sys
sys.path.append('/home/python_simulation/planners/')
from grid_based.A_star import A_star
from grid_based.Greedy_Best_First_Search import Greedy_Best_First_Search
from grid_based.Breadth_First_Search import Breadth_First_Search
from grid_based.Djikstra import Djikstra

from path_utilities import interpolate_path, filter_map, draw_map


import numpy as np
np.set_printoptions(threshold=sys.maxsize)

class Planners(Node):

    def __init__(self):
        super().__init__('Planners')
        self.publisher_path = self.create_publisher(Path, 'path', 1)
        self.publisher_map_filtered = self.create_publisher(OccupancyGrid,'map_filtered',1)

        self.subscription_goal = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.subscription_map = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)

        self.state_robot = np.zeros(3)
        
        self.map_origin = np.zeros(2)
        self.map = np.zeros((10,10))
        self.map_resolution = 0.05
        self.max_iteration = 1000
        self.visualize = False

        self.state_arrived = False
        self.map_arrived = False

        self.dt = 0.1
        self.safety = 5
        self.useFilteredMap = True

    def goal_callback(self, msg):
        if(self.state_arrived == True and self.map_arrived == True):
            goal = np.array([msg.pose.position.x, msg.pose.position.y])
            goal_shifted = np.zeros(2)
            goal_shifted[0] = goal[0] - self.map_origin[0]
            goal_shifted[1] = goal[1] - self.map_origin[1]

            state_shifted = np.zeros(3)
            state_shifted[0] = self.state_robot[0] - self.map_origin[0]
            state_shifted[1] = self.state_robot[1] - self.map_origin[1]

            print("Start: ", state_shifted)
            print("Goal: ", goal_shifted)

            state_shifted[0] = round(state_shifted[0],1)
            state_shifted[1] = round(state_shifted[1],1)

            goal_shifted[0] = round(goal_shifted[0],1)
            goal_shifted[1] = round(goal_shifted[1],1)


            #DRAW
            #draw_map(self.map, state_shifted, goal_shifted, self.map_resolution)


            #PLAN
            planner = A_star(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))
            path = planner.plan(self.max_iteration,self.visualize)
            

            #SPLINE
            spline, xs = interpolate_path(path, self.dt)


            #PUBLISH
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            for i in range(int(len(path)/0.1)):
                temp = spline(xs[i])
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = temp[0]*self.map_resolution + self.map_origin[0]
                poseStamped.pose.position.y = temp[1]*self.map_resolution + self.map_origin[1]

                poseStamped.header.frame_id = "odom"
                path_msg.poses.append(poseStamped)

            self.publisher_path.publish(path_msg)


    def tf_callback(self, msg):
        if(msg.transforms[0].child_frame_id == "base_footprint"):
            quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
                float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            
            self.state_robot[0] = msg.transforms[0].transform.translation.x
            self.state_robot[1] = msg.transforms[0].transform.translation.y
            self.state_robot[2] = euler[2]

            self.state_arrived = True


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

    
    def publish_map_filter(self,msg):
        d = 0
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if (self.map_filtered[i][j] == 254):
                    msg.data[d] = 0
                else:
                    msg.data[d] = 100      
                d = d+1
        self.publisher_map_filtered.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    print("###### Planner started ######")

    planner_node = Planners()

    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()