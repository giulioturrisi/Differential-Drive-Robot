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

sys.path.append('/home/python_scripts/controllers')
from io_linearization import IO_linearization # type: ignore
from nonlinear_lyapunov import Nonlinear_lyapunov # type: ignore
from approximate_linearization import Approximate_linearization # type: ignore
from dynamic_linearization import Dynamic_linearization # type: ignore


# Collection of multiple controllers ---------------------------------------


class Controllers(Node):
    def __init__(self):
        super().__init__('Controllers')
        # Path utilities ---------------------------------------
        self.path = []
        self.path_ready = False;

        # State utilities ---------------------------------------
        self.state_robot = np.zeros(3)


        # Controller choice ---------------------------------------
        self.which_controller = 1

        if(self.which_controller == 1):
            # Control utilities IO_linearization ---------------------------------------
            self.dt = 0.01
            self.k1 = 5
            self.k2 = 5
            self.b = 0.05
            self.controller = IO_linearization(b=self.b, k1=self.k1, k2=self.k2, dt=self.dt)
        elif(self.which_controller == 2):
            # Control utilities Approximate_linearization ---------------------------------------
            self.dt = 0.01
            self.k1 = 5
            self.k2 = 5
            self.k3 = 5
            self.controller = Approximate_linearization(k1=self.k1, k2=self.k2, k3=self.k3, dt=self.dt)
        elif(self.which_controller == 3):
            # Control utilities Nonlinear_lyapunov ---------------------------------------
            self.dt = 0.01
            self.k1 = 5
            self.k2 = 5
            self.k3 = 5
            self.controller = Nonlinear_lyapunov(k1=self.k1, k2=self.k2, k3=self.k3, dt=self.dt)
        elif(self.which_controller == 4):
            # Control utilities Dynamic_linearization ---------------------------------------
            self.dt = 0.01
            self.k1 = 2
            self.k2 = 15
            self.controller = Dynamic_linearization(k1=self.k1, k2=self.k2, dt=self.dt)


        # Subscribers and Publishers ---------------------------------------
        self.create_timer(self.dt, self.controller_callback)
        
        self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.subscription_path = self.create_subscription(Path,'path',self.getPath_callback,1)
        self.publisher_command =self.create_publisher(Twist,"cmd_vel", 1);


        # Sincronization with simulation ---------------------------------------
        self.enableSyncMode = Bool();
        self.enableSyncMode.data = True;
        self.publisher_enableSyncMode =self.create_publisher(Bool,"enableSyncMode", 1);
        self.publisher_enableSyncMode.publish(self.enableSyncMode)

        self.triggerNextStep = Bool();
        self.triggerNextStep.data = True;
        self.publisher_triggerNextStep = self.create_publisher(Bool,"triggerNextStep", 1);

        self.simStep_done = True
        self.subscription_simStep = self.create_subscription(Bool,'simulationStepDone',self.simStep_callback,1)






    # Controller callback ---------------------------------------
    def controller_callback(self):
        if(self.simStep_done):
            if(self.path_ready):
                # Compute control ---------------------------------------
                print("###############")
                print("state robot: ", self.state_robot)
                
                start_time = time.time()

                v, w = self.controller.compute_control(self.state_robot, self.path[0][0], self.path[0][1])
                
                print("control time: ", time.time()-start_time)

                # Publish Message ---------------------------------------
                self.publish_command(v,w)

                
                # Remove used reference point ---------------------------------------
                self.path.pop(0)
                if(len(self.path) == 0):
                    self.path_ready = False
                            
            else:
                # Zero control inputs ---------------------------------------
                v = 0
                w = 0
                self.publish_command(v, w)


            # Trigger next step Simulation ---------------------------------------
            self.simStep_done = False
            self.publisher_triggerNextStep.publish(self.triggerNextStep)
        

    # Path callback ---------------------------------------
    def getPath_callback(self, msg):
        # Get path from msg ---------------------------------------
        self.path = []
        for i in range (len(msg.poses)):
            x = msg.poses[i].pose.position.x;
            y = msg.poses[i].pose.position.y;
            self.path.insert(0,[x,y])

        self.path_ready = True;
        print("path received")
        self.controller.reset()


    # Robot state callback ---------------------------------------
    def tf_callback(self, msg):
        # Get state robot ---------------------------------------
        if(msg.transforms[0].child_frame_id == "base_footprint"):
            quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
                float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            
            self.state_robot[0] = msg.transforms[0].transform.translation.x #???
            self.state_robot[1] = msg.transforms[0].transform.translation.y
            self.state_robot[2] = euler[2] 


    # Perform next step simulation ---------------------------------------
    def simStep_callback(self,msg):
        self.simStep_done = True



def main(args=None):
    # Creation node ---------------------------------------
    rclpy.init(args=args)
    print("###### Controller started ######")

    controller_node = Controllers()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()