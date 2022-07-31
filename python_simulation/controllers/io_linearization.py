import random 
import math
import time

class IO_linearization:
    def __init__(self, b, k1, k2, dt):
        self.b = b
        self.k1 = k1
        self.k2 = k2
        self.previous_reference = None
        self.dt = dt
    
    def compute(self,initial_state, reference):
        reference_x = reference[0] 
        reference_y = reference[1]

        state_x = initial_state[0]
        state_y = initial_state[1]
        state_yaw = initial_state[2]


        if(self.previous_reference is not None):
            previous_reference_x = self.previous_reference[0]
            previous_reference_y = self.previous_reference[1] 
            ff_x = (reference_x - previous_reference_x)/self.dt;
            ff_y = (reference_y - previous_reference_y)/self.dt;
            
            self.previous_reference = reference 
        else:
            ff_x = 0.0
            ff_y = 0.0

        state_yaw = state_yaw + math.pi/2.;
        
        u1_io = ff_x + self.k1*(reference.x - (state_x + self.b*math.cos(state_yaw)))
        u2_io = ff_y + self.k2*(reference.y - (state_y + self.b*math.sin(state_yaw)))

        v = math.cos(state_yaw)*u1_io + math.sin(state_yaw)*u2_io;
        w = -math.sin(state_yaw)*u1_io/self.b + math.cos(state_yaw)*u2_io/self.b;


        return v, w