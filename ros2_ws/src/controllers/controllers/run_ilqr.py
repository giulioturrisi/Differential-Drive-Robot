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
from ilqr import iLQR 
sys.path.append('/home/ros2_ws/src/controllers/controllers')
from base_controller import Base_Controller




class Controller(Base_Controller):
    def __init__(self):
        super().__init__('ILQR')

        self.create_timer(self.dt, self.controller_callback)

        self.horizon = 30
        self.controller = iLQR(horizon=self.horizon, dt=self.dt)




    def controller_callback(self):
        if(self.simStep_done):
            if(self.path_ready):
                print("###############")
                print("state robot: ", self.state_robot)
                start_time = time.time()


                reference_x = []
                reference_y = []
                for i in range(self.horizon+1):
                    if(i < len(self.path)):
                        reference_x.append(self.path[i][0])
                        reference_y.append(self.path[i][1])
                    else:
                        reference_x.append(self.path[-1][0])
                        reference_y.append(self.path[-1][1])

                
                v, w = self.controller.compute_control(self.state_robot, reference_x, reference_y)

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
    rclpy.init(args=args)
    print("###### Controller started ######")

    controller_node = Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()