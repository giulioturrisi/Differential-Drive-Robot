import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

from std_msgs.msg import String, Bool # type: ignore
from geometry_msgs.msg import PoseStamped, Twist # type: ignore
from tf2_msgs.msg import TFMessage # type: ignore
from nav_msgs.msg import OccupancyGrid, Path # type: ignore
import tf_transformations # type: ignore

import copy
import matplotlib.pyplot as plt # type: ignore
import math
import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)

sys.path.append('/home/python_simulation/controllers')
from casadi_nmpc import Casadi_nmpc # type: ignore



class Controller(Node):
    def __init__(self):
        super().__init__('Casadi_NMPC')
        self.path = []
        self.path_ready = False;
        self.state_arrived = False
        self.dt = 0.02

        self.state_robot = np.zeros(3)

        self.create_timer(self.dt, self.controller_callback)
        
        self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.subscription_path = self.create_subscription(Path,'path',self.getPath_callback,1)

        self.publisher_command =self.create_publisher(Twist,"cmd_vel", 2);

        self.horizon = 50
        self.controller = Casadi_nmpc(self.horizon,[],[], self.dt)
        #self.controller = Casadi_nmpc(self.horizon,[],[], 0.01)


        self.enableSyncMode = Bool();
        self.enableSyncMode.data = True;
        self.publisher_enableSyncMode =self.create_publisher(Bool,"enableSyncMode", 1);
        #self.publisher_enableSyncMode.publish(self.enableSyncMode)


        self.triggerNextStep = Bool();
        self.triggerNextStep.data = True;
        #self.publisher_triggerNextStep = self.create_publisher(Bool,"triggerNextStep", 1);

        self.M_PI = 3.14159265358979323846




    def controller_callback(self):
        if(self.path_ready and self.state_arrived):
            print("###############")
            print("state robot: ", self.state_robot)
            #start_time = time.time()


            reference_x = []
            reference_y = []
            for i in range(self.horizon):
                if(i < len(self.path)):
                    reference_x.append(self.path[i][0])
                    reference_y.append(self.path[i][1])
                    #reference_x.append(0.2)
                    #reference_y.append(0.1)
                else:
                    #reference_x.append(0.0)
                    #reference_y.append(0.0)
                    reference_x.append(self.path[-1][0])
                    reference_y.append(self.path[-1][1])
                    #reference_x.append(0.2)
                    #reference_y.append(0.1)

            self.controller.initialize_casadi()
            v, w = self.controller.compute_mpc(self.state_robot, reference_x, reference_y)
            #x_new, y_new, theta_new =  self.controller.compute_mpc(self.state_robot, reference_x, reference_y)

            #print("command: ", [x_new, y_new])
            
            #print("control time: ", time.time()-start_time)

            commanded_vel = Twist();
            commanded_vel.linear.x = v;
            commanded_vel.angular.z = w;

            #commanded_vel.linear.x = x_new;
            #commanded_vel.linear.y = y_new;
            #commanded_vel.linear.z = theta_new;
            self.publisher_command.publish(commanded_vel);
            
            self.path.pop(0)
            if(len(self.path) == 0):
                self.path_ready = False

            #self.publisher_triggerNextStep.publish(self.triggerNextStep)
            self.state_arrived = False
            
            
        else:
            commanded_vel = Twist();
            commanded_vel.linear.x = 0.0;
            commanded_vel.angular.z = 0.0;
            self.publisher_command.publish(commanded_vel);
        

        
        

        


    def getPath_callback(self, msg):
        for i in range (len(msg.poses)):
            x = msg.poses[i].pose.position.x;
            y = msg.poses[i].pose.position.y;
            self.path.insert(0,[x,y])
            #self.path.append([x,y])

        self.path_ready = True;
        print("path received")



    def tf_callback(self, msg):
        if(msg.transforms[0].child_frame_id == "base_footprint"):
            quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
                float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            
            self.state_robot[0] = msg.transforms[0].transform.translation.x #???
            self.state_robot[1] = msg.transforms[0].transform.translation.y
            self.state_robot[2] = euler[2] 


            self.state_arrived = True



def main(args=None):
    rclpy.init(args=args)
    print("###### Controller started ######")

    controller_node = Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()