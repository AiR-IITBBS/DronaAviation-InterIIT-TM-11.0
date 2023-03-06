from numpy import array as arr

# permissible_error_throttle = 0.03
# permissible_error_location = 0.05


class PIDController:

    # initialises the values & properties for PIDController
    def __init__(self, target_position, k_values, range):

        self.origin_position = [0.0, 0.0, 0.0]
        self.drone_position = [0.0, 0.0, 0.0]
        self.target_position = target_position

        self.Kp = k_values[0]  # TPR
        self.Ki = k_values[1]
        self.Kd = k_values[2]

        self.error = [0.0, 0.0, 0.0]  # XYZ
        self.prev_value = [0.0, 0.0, 0.0]
        self.integral_error = [0.0, 0.0, 0.0]

        self.out_pitch = 0.0
        self.out_roll = 0.0
        self.out_throttle = 0.0

        self.max_values = [range[0][1], range[1][1], range[2][1]]  # TPR
        self.min_values = [range[0][0], range[1][0], range[2][0]]

        self.current_state = [1500, 1500, 1500]

    # calculates the state values(TPR) as per drone's current_position
    def calculate_state(self, current_position):

        self.drone_position = current_position

        self.error[0] = self.drone_position[0] - self.target_position[0]
        self.error[1] = self.drone_position[1] - self.target_position[1]
        self.error[2] = self.drone_position[2] - self.target_position[2]

        self.integral_error[0] = self.integral_error[0] + self.error[0]
        self.integral_error[1] = self.integral_error[1] + self.error[1]
        self.integral_error[2] = self.integral_error[2] + self.error[2]

        self.out_throttle = int(self.Kp[0] * self.error[2] + self.Kd[0]*(
            self.error[2]-self.prev_value[2]) + self.Ki[0]*self.integral_error[2])
        self.out_roll = int(self.Kp[2] * self.error[1] + self.Kd[2]*(
            self.error[1]-self.prev_value[1]) + self.Ki[2]*self.integral_error[1])
        self.out_pitch = int(self.Kp[1] * self.error[0] + self.Kd[1]*(
            self.error[0]-self.prev_value[0]) + self.Ki[1]*self.integral_error[0])

        self.drone_throttle = 1500 + self.out_throttle
        self.drone_roll = 1500 + self.out_roll
        self.drone_pitch = 1500 + self.out_pitch

        self.prev_value[0] = self.error[0]
        self.prev_value[1] = self.error[1]
        self.prev_value[2] = self.error[2]

        self.clamp_state_values()

        # print(self.drone_pitch,self.drone_roll,self.drone_throttle)
        return [self.drone_throttle, self.drone_pitch, self.drone_roll]

    def set_target(self, checkpoint):  # sets the target to the given position(checkpoint)
        # Note: checkpoint is a list of the form [x,y,z]
        self.target_position = checkpoint

    def get_error(self):  # returns the current error
        return self.error

    # clamps the state values according to  "range"(given as input during initiation)
    def clamp_state_values(self):
        if (self.drone_roll > self.max_values[2]):
            self.drone_roll = self.max_values[2]
            self.integral_error[1] -= (self.drone_roll -
                                       self.max_values[2])/self.Ki[2]
        if (self.drone_pitch > self.max_values[1]):
            self.drone_pitch = self.max_values[1]
            self.integral_error[0] -= (self.drone_pitch -
                                       self.max_values[1])/self.Ki[1]
        if (self.drone_throttle > self.max_values[0]):
            self.drone_throttle = self.max_values[0]
            self.integral_error[2] -= (self.drone_throttle -
                                       self.max_values[0])/self.Ki[0]
        if (self.drone_roll < self.min_values[2]):
            self.drone_roll = self.min_values[2]
            self.integral_error[1] -= (self.drone_roll -
                                       self.min_values[2])/self.Ki[2]
        if (self.drone_pitch < self.min_values[1]):
            self.drone_pitch = self.min_values[1]
            self.integral_error[0] -= (self.drone_pitch -
                                       self.min_values[1])/self.Ki[1]
        if (self.drone_throttle < self.min_values[0]):
            self.drone_throttle = self.min_values[0]
            self.integral_error[2] -= (self.drone_throttle -
                                       self.min_values[0])/self.Ki[0]
