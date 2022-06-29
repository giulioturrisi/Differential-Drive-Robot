import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid, Path

import tf_transformations

import copy


import sys
sys.path.append('/home/python_simulation/planners/A_star')
from A_star import A_star
sys.path.append('/home/python_simulation/planners/Greedy_Best_First_Search')
from Greedy_Best_First_Search import Greedy_Best_First_Search
sys.path.append('/home/python_simulation/planners/Breadth_First_Search')
from Breadth_First_Search import Breadth_First_Search
sys.path.append('/home/python_simulation/planners/Djikstra')
from Djikstra import Djikstra


import numpy as np
np.set_printoptions(threshold=sys.maxsize)

class Planners(Node):

    def __init__(self):
        super().__init__('Planners')
        self.publisher_path = self.create_publisher(Path, 'path', 1)

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

    def goal_callback(self, msg):
        if(self.state_arrived == True and self.map_arrived == True):
            goal = np.array([msg.pose.position.x, msg.pose.position.y])
            goal_shifted = np.zeros(2)
            goal_shifted[0] = round(goal[0] - self.map_origin[0],1)
            goal_shifted[1] = round(goal[1] - self.map_origin[1],1)

            state_shifted = np.zeros(3)
            state_shifted[0] = round(self.state_robot[0] - self.map_origin[0],1)
            state_shifted[1] = round(self.state_robot[1] - self.map_origin[1],1)

            print("Start: ", state_shifted)
            print("Goal: ", goal_shifted)

            planner = Breadth_First_Search(state_shifted, goal_shifted, self.map, round(self.map_resolution,2))
            path = planner.plan(self.max_iteration,self.visualize)
            print("path", path)

            path_msg = Path()
            path_msg.header.frame_id = "odom"

            for i in range(np.shape(path)[0]):
                temp = path.pop()
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = temp[0] + self.map_origin[0]
                poseStamped.pose.position.y = temp[1] + self.map_origin[1]

                print("poseStamped",poseStamped)
                print("poseStamped.pose.position.x",poseStamped.pose.position.x)
                print("path[i][0]",path[i][0])
                print("i",i)
                print("path[i]:",temp)
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
                self.map[i][j] = 255 if (msg.data[d] < 10 and msg.data[d] >= 0) else 0
                self.map[i][j] = 255
                d = d+1

        self.map_origin[0] = msg.info.origin.position.x
        self.map_origin[1] = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution

        self.map_arrived = True

def main(args=None):
    rclpy.init(args=args)
    print("###### Planner started ######")

    planner_node = Planners()


    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()