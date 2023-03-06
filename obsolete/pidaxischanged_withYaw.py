from numpy import array as arr

# permissible_error_throttle = 0.03
# permissible_error_location = 0.05

class PIDController:

    def __init__(self , target_position, k_values, range): # NEED TO ADD MORE SPACE IN TARGET POSITION AND K_VALUES FOR ACCOMODATING THE YAW VALUES***********************************************

        self.origin_position = [0.0, 0.0, 0.0,0.0]
        self.drone_position = [0.0,0.0,0.0,0.0]
        self.target_position = target_position 

        self.Kp = k_values[0] #TPR
        self.Ki = k_values[1]
        self.Kd = k_values[2]

        self.error = [0.0,0.0,0.0,0.0] #XYZ and angle
        self.prev_value = [0.0,0.0,0.0,0.0]
        self.integral_error = [0.0,0.0,0.0,0.0]

        self.out_pitch = 0.0
        self.out_roll = 0.0
        self.out_throttle = 0.0
        self.out_yaw = 0.0

        self.max_values = [range[0][1],range[1][1],range[2][1],range[3][1]] #TPR
        self.min_values = [range[0][0],range[1][0],range[2][0],range[3][0]]
        #NEED TO ADD MORE VALUES IN MAX AND MIN FOR YAW****************************************

        self.current_state = [1500, 1500, 1500, 1500]

    def calculate_state(self , current_position): # NEED TO ADD MORE SPACE IN CURRENT_POSITION FOR ACCOMODATING THE INPUT OR TARGETED VALUE OF YAW

        self.drone_position = current_position

        self.error[0] = self.drone_position[0] - self.target_position[0]
        self.error[1] = self.drone_position[1] - self.target_position[1]
        self.error[2] = self.drone_position[2] - self.target_position[2]
        self.error[3] = self.drone_position[3] - self.target_position[3] # Angle error for yaw

        self.integral_error[0] = self.integral_error[0] + self.error[0]
        self.integral_error[1] = self.integral_error[1] + self.error[1]
        self.integral_error[2] = self.integral_error[2] + self.error[2]
        self.integral_error[3] = self.integral_error[3] + self.error[3]

        self.out_throttle = int(self.Kp[0] * self.error[2] + self.Kd[0]*(self.error[2]-self.prev_value[2]) + self.Ki[0]*self.integral_error[2])
        self.out_roll = int(self.Kp[2] * self.error[1] + self.Kd[2]*(self.error[1]-self.prev_value[1]) + self.Ki[2]*self.integral_error[1])
        self.out_pitch = int(self.Kp[1] * self.error[0] + self.Kd[1]*(self.error[0]-self.prev_value[0]) + self.Ki[1]*self.integral_error[0])
        self.out_yaw = int(self.Kp[3] * self.error[3] + self.Kd[3]*(self.error[3]-self.prev_value[3]) + self.Ki[3]*self.integral_error[3])

        self.drone_throttle = 1500 + self.out_throttle
        self.drone_roll = 1500 + self.out_roll 
        self.drone_pitch = 1500 + self.out_pitch
        self.drone_yaw = 1500 + self.out_yaw

        self.prev_value[0] = self.error[0]
        self.prev_value[1] = self.error[1]
        self.prev_value[2] = self.error[2]
        self.prev_value[3] = self.error[3]

        self.clamp_state_values()

        # print(self.drone_pitch,self.drone_roll,self.drone_throttle)
        return [self.drone_throttle , self.drone_pitch , self.drone_roll , self.drone_yaw]
        #NEED TO CHECK THE OUTPUTS************************************************

    def set_target(self,checkpoint):
        self.target_position = checkpoint

    def get_error(self):
        return self.error
        
    def clamp_state_values(self):
        if(self.drone_roll > self.max_values[2] ):
            self.drone_roll = self.max_values[2]
            self.integral_error[1] -= (self.drone_roll - self.max_values[2])/self.Ki[2]
        if(self.drone_pitch > self.max_values[1] ):
            self.drone_pitch = self.max_values[1]
            self.integral_error[0] -= (self.drone_pitch - self.max_values[1])/self.Ki[1]
        if(self.drone_throttle > self.max_values[0] ):
            self.drone_throttle = self.max_values[0]
            self.integral_error[2] -= (self.drone_throttle - self.max_values[0])/self.Ki[0]
        if(self.drone_yaw > self.max_values[3] ):
            self.drone_yaw = self.max_values[3]
            self.integral_error[3] -= (self.drone_yaw - self.max_values[3])/self.Ki[3]
        if( self.drone_roll < self.min_values[2]):
            self.drone_roll =  self.min_values[2]
            self.integral_error[1] -= (self.drone_roll - self.min_values[2])/self.Ki[2]
        if( self.drone_pitch < self.min_values[1]):
            self.drone_pitch =  self.min_values[1]
            self.integral_error[0] -= (self.drone_pitch - self.min_values[1])/self.Ki[1]
        if( self.drone_throttle < self.min_values[0]):
            self.drone_throttle =  self.min_values[0]
            self.integral_error[2] -= (self.drone_throttle - self.min_values[0])/self.Ki[0]
        if( self.drone_yaw < self.min_values[3]):
            self.drone_yaw =  self.min_values[3]
            self.integral_error[3] -= (self.drone_yaw - self.min_values[3])/self.Ki[3]


