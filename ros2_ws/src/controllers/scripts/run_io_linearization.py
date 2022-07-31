import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid, Path
import tf_transformations

import copy
import matplotlib.pyplot as plt
import math
import time
import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)

sys.path.append('/home/python_simulation/controllers')
from io_linearization import IO_linearization




class Controller(Node):
    def __init__(self):
        super().__init__('IO_linearization')
        # Path utilities ---------------------------------------
        self.path = []
        self.path_ready = False;

        # State utilities ---------------------------------------
        self.state_arrived = False
        self.state_robot = np.zeros(3)

        # Control utilities ---------------------------------------
        self.dt = 0.01
        self.k1 = 5
        self.k2 = 5
        self.b = 0.01
        self.controller = IO_linearization(b=self.b, k1=self.k1, k2=self.k2, dt=self.dt)


        # Subscribers and Publishers ---------------------------------------
        self.create_timer(self.dt, self.controller_callback)
        
        self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.subscription_path = self.create_subscription(Path,'path',self.getPath_callback,1)
        self.publisher_command =self.create_publisher(Twist,"cmd_vel", 2);


        # Sincronization with simulation ---------------------------------------
        self.enableSyncMode = Bool();
        self.enableSyncMode.data = True;
        self.publisher_enableSyncMode =self.create_publisher(Bool,"enableSyncMode", 1);
        #self.publisher_enableSyncMode.publish(self.enableSyncMode)


        self.triggerNextStep = Bool();
        self.triggerNextStep.data = True;
        #self.publisher_triggerNextStep = self.create_publisher(Bool,"triggerNextStep", 1);






    def controller_callback(self):
        if(self.path_ready and self.state_arrived):
            # Compute control ---------------------------------------
            print("###############")
            print("state robot: ", self.state_robot)
            start_time = time.time()

            reference = [self.path[0][0], self.path[0][1]]

            self.controller.initialize_casadi()
            v, w = self.controller.compute(self.state_robot, reference)
            
            print("control time: ", time.time()-start_time)

            # Publish Message ---------------------------------------
            commanded_vel = Twist();
            commanded_vel.linear.x = v;
            commanded_vel.angular.z = w;
            self.publisher_command.publish(commanded_vel);
            
            # Remove used reference point ---------------------------------------
            self.path.pop(0)
            if(len(self.path) == 0):
                self.path_ready = False

            # Trigger next step Simulation ---------------------------------------
            #self.publisher_triggerNextStep.publish(self.triggerNextStep)
            self.state_arrived = False
            
            
        else:
            # Zero control inputs ---------------------------------------
            commanded_vel = Twist();
            commanded_vel.linear.x = 0.0;
            commanded_vel.angular.z = 0.0;
            self.publisher_command.publish(commanded_vel);
        



    def getPath_callback(self, msg):
        # Get path from msg ---------------------------------------
        for i in range (len(msg.poses)):
            x = msg.poses[i].pose.position.x;
            y = msg.poses[i].pose.position.y;
            self.path.insert(0,[x,y])
            #self.path.append([x,y])

        self.path_ready = True;
        print("path received")



    def tf_callback(self, msg):
        # Get state robot ---------------------------------------
        if(msg.transforms[0].child_frame_id == "base_footprint"):
            quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
                float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            
            self.state_robot[0] = msg.transforms[0].transform.translation.x #???
            self.state_robot[1] = msg.transforms[0].transform.translation.y
            self.state_robot[2] = euler[2] 


            self.state_arrived = True



def main(args=None):
    # Creation node ---------------------------------------
    rclpy.init(args=args)
    print("###### Controller started ######")

    controller_node = Controller()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()