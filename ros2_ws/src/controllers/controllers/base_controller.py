import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

from std_msgs.msg import String, Bool # type: ignore
from geometry_msgs.msg import PoseStamped, Twist # type: ignore
from tf2_msgs.msg import TFMessage # type: ignore
from nav_msgs.msg import OccupancyGrid, Path # type: ignore
import tf_transformations # type: ignore

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

import copy
import matplotlib.pyplot as plt # type: ignore
import math
import time
import sys
import numpy as np # type: ignore
np.set_printoptions(threshold=sys.maxsize)



# Shell for all the controllers ----------------------------------------------
class Base_Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        self.path = []
        self.path_ready = False;
        self.state_arrived = False
        self.dt = 0.01
        self.state_robot = np.zeros(3)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.controller = None

        
        # Publisher and Subscribers -------------------------------------------
        #self.subscription_tf = self.create_subscription(TFMessage,'tf',self.tf_callback,1)
        self.create_timer(self.dt/2., self.tf_callback)
        self.subscription_path = self.create_subscription(Path,'path',self.getPath_callback,1)
        self.subscription_cmd_vel = self.create_subscription(Twist,"cmd_vel", self.getVel_callback, 1);

        self.publisher_command = self.create_publisher(Twist,"control_vel", 2);


        # Sincronization with simulation ---------------------------------------
        self.enableSyncMode = Bool();
        self.enableSyncMode.data = False # Put this to True if you want to synchronize with CoppeliaSim
        self.publisher_enableSyncMode = self.create_publisher(Bool,"enableSyncMode", 1);
        self.publisher_enableSyncMode.publish(self.enableSyncMode)

        self.requested_step = False


        self.triggerNextStep = Bool();
        self.triggerNextStep.data = True;
        self.publisher_triggerNextStep = self.create_publisher(Bool,"triggerNextStep", 1);

        self.steps_without_answer = 0
        self.simStep_done = True
        self.subscription_simStep = self.create_subscription(Bool,'simulationStepDone',self.simStep_callback,1)


    # Path callback ----------------------------------------------
    def getPath_callback(self, msg):
        self.path = []
        for i in range (len(msg.poses)):
            x = msg.poses[i].pose.position.x;
            y = msg.poses[i].pose.position.y;
            self.path.insert(0,[x,y])
            

        self.path_ready = True;
        print("Path received!")
        self.controller.reset()

    # Velocity received by the joystick ---------------------------
    def getVel_callback(self, msg):
        self.path_ready = False
        v = msg.linear.x/10.
        w = msg.angular.z
        self.publish_command(v, w)
        self.controller.reset()
        


    # Trasformation callback ---------------------------------------
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
            
            self.state_arrived = True
        except TransformException as ex:
            return


    # Simulation step done flag -------------------------------------
    def simStep_callback(self,msg):
        self.simStep_done = True
        self.requested_step = False


    # Request for next step simulation ------------------------------
    def triggerNextStep_Sim(self,):
        if(self.enableSyncMode.data == True):
            self.simStep_done = False
        self.publisher_triggerNextStep.publish(self.triggerNextStep)


    # Publish the motor commands ------------------------------------
    def publish_command(self, v, w):
        print("state robot: ", self.state_robot)
        print("control actions: ", [v,w])
        commanded_vel = Twist()
        commanded_vel.linear.x = float(v)
        commanded_vel.angular.z = float(w)
        self.publisher_command.publish(commanded_vel);



def main(args=None):
    rclpy.init(args=args)

    controller_node = Base_Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()