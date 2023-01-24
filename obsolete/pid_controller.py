from numpy import array as arr

Drone_roll=0.0
Drone_pitch=0.0
Drone_throttle=0.0

Drone_error = [0,0,0]
permissible_error_throttle = 0.03
permissible_error_location = 0.05

class pid:

    def __init__(self , set_pt , origin):

        self.drone_position = [0.0,0.0,0.0]
        # self.setpoint = list(arr(set_pt) - arr(origin))
        self.setpoint = set_pt

        self.Kp = [100 , 100 , 350]
        self.Ki = [0 , 0 , 0]
        self.Kd = [0 , 0 , 0]

        self.error = [0.0,0.0,0.0]
        self.prev_value = [0.0,0.0,0.0]
        self.sum_error = [0.0,0.0,0.0]
        self.out_pitch = 0.0
        self.out_roll = 0.0
        self.out_throttle = 0.0
        self.max_values = [1600,1600,1650]
        self.min_values = [1400,1400,1200]

        self.drone_roll = 1500
        self.drone_pitch = 1500
        self.drone_throttle = 1500

    def pid(self , curr_pos):

        global Drone_pitch, Drone_roll, Drone_throttle , Drone_error

        self.drone_position = curr_pos

        self.error[0] = self.drone_position[0] - self.setpoint[0]
        self.error[1] = self.drone_position[1] - self.setpoint[1]
        self.error[2] = self.drone_position[2] - self.setpoint[2]

        self.out_roll = int(self.Kp[0] * self.error[0] + self.Kd[0]*(self.error[0]-self.prev_value[0]) + self.Ki[0]*self.sum_error[0])
        self.out_pitch = int(self.Kp[1] * self.error[1] + self.Kd[1]*(self.error[1]-self.prev_value[1]) + self.Ki[1]*self.sum_error[1])
        self.out_throttle = int(self.Kp[2] * self.error[2] + self.Kd[2]*(self.error[2]-self.prev_value[2]) + self.Ki[2]*self.sum_error[2])

        self.drone_roll = 1500 - self.out_roll
        self.drone_pitch = 1500 + self.out_pitch
        # self.drone_throttle = 1500 + self.out_throttle
        self.drone_throttle = 1500 + self.out_throttle

        if(self.drone_roll > self.max_values[0] ):
            self.drone_roll = self.max_values[0]
        if(self.drone_pitch > self.max_values[1] ):
            self.drone_pitch = self.max_values[1]
        if(self.drone_throttle > self.max_values[2] ):
            self.drone_throttle = self.max_values[2]
        if( self.drone_roll < self.min_values[0]):
            self.drone_roll =  self.min_values[0]
        if( self.drone_pitch < self.min_values[1]):
            self.drone_pitch =  self.min_values[1]
        if( self.drone_throttle < self.min_values[2]):
            self.drone_throttle =  self.min_values[2]

        self.prev_value[0] = self.error[0]
        self.prev_value[1] = self.error[1]
        self.prev_value[2] = self.error[2]
        self.sum_error[0] = self.sum_error[0] + self.error[0]
        self.sum_error[1] = self.sum_error[1] + self.error[1]
        self.sum_error[2] = self.sum_error[2] + self.error[2]
        # Need to look into anti windup for limiting integral term.

        Drone_roll=self.drone_roll
        Drone_pitch=self.drone_pitch
        Drone_throttle=self.drone_throttle

        Drone_error = [ self.error[0] , self.error[1] , self.error[2] ]

        print(Drone_pitch,Drone_roll,Drone_throttle)

        return [Drone_throttle , Drone_pitch , Drone_roll]
        

