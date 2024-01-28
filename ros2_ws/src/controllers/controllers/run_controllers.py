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

import threading

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

sys.path.append(dir_path + '/../../../../python_scripts/controllers/acados')
from acados_nmpc import Acados_NMPC 
sys.path.append(dir_path + '/../../../../python_scripts/controllers')
from dynamic_linearization import Dynamic_linearization 
from ilqr import iLQR 
from io_linearization import IO_linearization
from io_linearization_mpc import IO_linearization_MPC
from nonlinear_lyapunov import Nonlinear_lyapunov
from approximate_linearization import Approximate_linearization
from casadi_nmpc import Casadi_NMPC
from predictive_sampling import Sampling_MPC

sys.path.append(dir_path)
from base_controller import Base_Controller



class Controller(Base_Controller):
    def __init__(self):
        super().__init__('Controllers')

        self.create_timer(self.dt, self.controller_callback)

        self.which_controller = 2
        self.select_controller()

        

        # Interactive Command Line ----------------------------
        t1 = threading.Thread(target=self.interactive_command_line)
        t1.daemon = True
        t1.start()


    def controller_callback(self):
        if(self.simStep_done):
            if(self.path_ready):
                print("###############")
                start_time = time.time()

                reference_x = []
                reference_y = []
                if(self.horizon == 0):
                    reference_x = self.path[0][0]
                    reference_y = self.path[0][1]
                else:
                    for i in range(self.horizon+1):
                        if(i < len(self.path)):
                            reference_x.append(self.path[i][0])
                            reference_y.append(self.path[i][1])
                            #reference_x.append(0.1)
                            #reference_y.append(1)
                        else:
                            reference_x.append(self.path[-1][0])
                            reference_y.append(self.path[-1][1])
                            #reference_x.append(0.1)
                            #reference_y.append(1)

                v, w = self.controller.compute_control(self.state_robot, reference_x, reference_y)

                print("control time: ", time.time()-start_time)

                # Publish Message ---------------------------------------
                self.publish_command(v,w)

                
                # Remove used reference point ---------------------------------------
                self.path.pop(0)
                if(len(self.path) == 0):
                    self.path_ready = False
                    self.publish_command(0,0)
                            


            # Trigger next step Simulation ---------------------------------------
            self.triggerNextStep_Sim()
            self.steps_without_answer = 0
            self.requested_step = True
        
        if(self.requested_step == True):
            self.steps_without_answer = self.steps_without_answer + 1
            if(self.steps_without_answer > 100):
                self.triggerNextStep_Sim()
                self.steps_without_answer = 0



    def select_controller(self, ):
        if(self.which_controller == 1):
            self.horizon = 20
            self.controller = Acados_NMPC(self.horizon, self.dt)
        elif(self.which_controller == 2):
            self.horizon = 20
            self.controller = Casadi_NMPC(self.horizon, self.dt)
        elif(self.which_controller == 3):
            self.horizon = 20
            self.controller = iLQR(horizon=self.horizon, dt=self.dt)
        elif(self.which_controller == 4):
            self.horizon = 0
            self.k1 = 2
            self.k2 = 15
            self.controller = Dynamic_linearization(k1=self.k1, k2=self.k2, dt=self.dt)
        elif(self.which_controller == 5):
            self.horizon = 0
            self.k1 = 5
            self.k2 = 5
            self.b = 0.05
            self.controller = IO_linearization(b=self.b, k1=self.k1, k2=self.k2, dt=self.dt)
        elif(self.which_controller == 6):
            self.horizon = 20
            self.b = 0.05
            self.controller = IO_linearization_MPC(self.horizon, b=self.b, dt=self.dt)
        elif(self.which_controller == 7):
            self.horizon = 0
            self.k1 = 5
            self.k2 = 5
            self.k3 = 5
            self.controller = Nonlinear_lyapunov(k1=self.k1, k2=self.k2, k3=self.k3, dt=self.dt)
        elif(self.which_controller == 8):
            self.horizon = 0
            self.k1 = 5
            self.k2 = 5
            self.k3 = 5
            self.controller = Approximate_linearization(k1=self.k1, k2=self.k2, k3=self.k3, dt=self.dt)
        elif(self.which_controller == 9):
            self.horizon = 20
            self.controller = Sampling_MPC(horizon=self.horizon, dt=self.dt, init_jax = True, linear = False)
        else:
            print("Wrong input")
            self.list_controllers()

    def list_controllers(self, ):
        print("---- You can change the controller by pressing ----")
        print("1: Acados NMPC")
        print("2: Casadi NMPC")
        print("3: ILQR")
        print("4: Dynamic Linearization")
        print("5: IO Linearization")
        print("6: IO Linearization MPC")
        print("7: Nonlinear Lyapunov")
        print("8: Approximate Linearization")
        print("9: Predictive Sampling MPC")
        print("---------------------------------------------------")


    def interactive_command_line(self, ):
        self.list_controllers()
        while True:
            new_controller = input(">>> ")
            if(new_controller == "1"):
                self.which_controller = 1
                self.select_controller()
                print("## Controller started with Acados NMPC ##")
            elif(new_controller == "2"):
                self.which_controller = 2
                self.select_controller()
                print("## Controller started with Casadi NMPC ##")
            elif(new_controller == "3"):
                self.which_controller = 3
                self.select_controller()
                print("## Planner started with ILQR ##")
            elif(new_controller == "4"):
                self.which_controller = 4
                self.select_controller()
                print("## Controller started with Dynamic Linearization ##")
            elif(new_controller == "5"):
                self.which_controller = 5
                self.select_controller()
                print("## Controller started with IO Linearization ##")
            elif(new_controller == "6"):
                self.which_controller = 6
                self.select_controller()
                print("## Controller started with IO Linearization MPC ##")
            elif(new_controller == "7"):
                self.which_controller = 7
                self.select_controller()
                print("## Controller started with Nonlinear Lyapunov ##")
            elif(new_controller == "8"):
                self.which_controller = 8
                self.select_controller()
                print("## Controller started with Approximate Linearization ##")
            elif (new_controller == "9"):
                self.which_controller = 9
                self.select_controller()
                print("## Controller started with Predictive Sampling MPC ##")
            else:
                print("Wrong input")
                self.list_controllers()
                continue



def main(args=None):
    rclpy.init(args=args)
    print("## Controller started with Casadi NMPC ##")

    controller_node = Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()