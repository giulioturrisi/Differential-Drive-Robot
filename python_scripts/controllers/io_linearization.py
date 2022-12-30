import random 
import math
import time

class IO_linearization:
    """Class for the IO_linearization control law
    """
    def __init__(self, b, k1, k2, dt):
        """Init func
        Args:
            b (float): distance from the center of the robot where to perform the linearization
            k1 (float): gain for x error
            k2 (float): gain for y error
            dt (float): sampling time
        """
        self.b = b
        self.k1 = k1
        self.k2 = k2

        self.dt = dt


    def reset(self,):
        """Every control class should have a reset function
        """
        self.previous_reference = None
    

    def compute_control(self,initial_state, reference_x, reference_y):
        """Compute the control actions
        Args:
            initial_state (np.array): actual state of the robot
            reference_x (np.array): x reference for the robot
            reference_y (np.array): y reference for the robot
        Returns:
            (np.array): control actions
        """
        state_x = initial_state[0]
        state_y = initial_state[1]
        state_yaw = initial_state[2]


        if(self.previous_reference is not None):
            previous_reference_x = self.previous_reference[0]
            previous_reference_y = self.previous_reference[1] 
            ff_x = (reference_x - previous_reference_x)/self.dt;
            ff_y = (reference_y - previous_reference_y)/self.dt;
            
            self.previous_reference = [reference_x, reference_y] 
        else:
            ff_x = 0.0
            ff_y = 0.0

        ff_x = 0.0
        ff_y = 0.0

        state_yaw = state_yaw + 0*math.pi/2.;
        
        u1_io = ff_x + self.k1*(reference_x - (state_x + self.b*math.cos(state_yaw)))
        u2_io = ff_y + self.k2*(reference_y - (state_y + self.b*math.sin(state_yaw)))

        v = math.cos(state_yaw)*u1_io + math.sin(state_yaw)*u2_io;
        w = (-math.sin(state_yaw)*u1_io/self.b) + (math.cos(state_yaw)*u2_io/self.b);


        return v, w