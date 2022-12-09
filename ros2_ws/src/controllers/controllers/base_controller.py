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




class Base_Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        self.path = []
        self.path_ready = False;
        self.state_arrived = False
        self.dt = 0.01

        self.controller = None

        self.state_robot = np.zeros(3)
        
        self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.subscription_path = self.create_subscription(Path,'path',self.getPath_callback,1)
        self.subscription_cmd_vel = self.create_subscription(Twist,"cmd_vel", self.getVel_callback, 1);

        self.publisher_command = self.create_publisher(Twist,"control_vel", 2);


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



    def getPath_callback(self, msg):
        self.path = []
        for i in range (len(msg.poses)):
            x = msg.poses[i].pose.position.x;
            y = msg.poses[i].pose.position.y;
            self.path.insert(0,[x,y])
            #self.path.append([x,y])

        self.path_ready = True;
        print("Path received!")
        self.controller.reset()

    # coming from joystick!
    def getVel_callback(self, msg):
        self.path_ready = False
        self.publish_command(msg.linear.x, msg.linear.z)
        self.controller.reset()
        



    def tf_callback(self, msg):
        if(msg.transforms[0].child_frame_id == "base_footprint"):
            quaternion = [float(msg.transforms[0].transform.rotation.x), float(msg.transforms[0].transform.rotation.y), 
                float(msg.transforms[0].transform.rotation.z), float(msg.transforms[0].transform.rotation.w)]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            
            self.state_robot[0] = msg.transforms[0].transform.translation.x #???
            self.state_robot[1] = msg.transforms[0].transform.translation.y
            self.state_robot[2] = euler[2] 


            self.state_arrived = True


    def simStep_callback(self,msg):
        self.simStep_done = True


    def triggerNextStep_Sim(self,):
        self.simStep_done = False
        self.publisher_triggerNextStep.publish(self.triggerNextStep)


    def publish_command(self, v, w):
        commanded_vel = Twist()
        commanded_vel.linear.x = np.float(v)
        commanded_vel.angular.z = np.float(w)
        self.publisher_command.publish(commanded_vel);



def main(args=None):
    rclpy.init(args=args)

    controller_node = Base_Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()