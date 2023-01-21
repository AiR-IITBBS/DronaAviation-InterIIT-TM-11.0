from Communication import Drone
from utils import ARUCO_DICT, print_coordinates
import pid_controller as PID
import time as tm
import numpy as np
import matplotlib.pyplot as plt

aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
calibration_matrix_path = "calibration_data/calibration_matrix.npy"
distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

drone = Drone("192.168.4.1", 23, 5, debug=True) #creates drone object, set debug to true to get console output on every action.
drone.connect() #starts looping in a separate thread
drone.takeoff()

flight_duration = 3

start = tm.time()

while(tm.time()-start<flight_duration):
    drone.set_state( 1500 , 1500 , 1100 )
    tm.sleep(0.022)

drone.land()
drone.disconnect()