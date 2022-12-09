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
from approximate_linearization import Approximate_linearization # type: ignore
sys.path.append('/home/ros2_ws/src/controllers/controllers')
from base_controller import Base_Controller

class Controller(Base_Controller):
    def __init__(self):
        super().__init__('Approximate_linearization')

        self.k1 = 5
        self.k2 = 5
        self.k3 = 5
        self.controller = Approximate_linearization(k1=self.k1, k2=self.k2, k3=self.k3, dt=self.dt)

        self.create_timer(self.dt, self.controller_callback)


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



            # Trigger next step Simulation ---------------------------------------
            self.triggerNextStep_Sim()



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