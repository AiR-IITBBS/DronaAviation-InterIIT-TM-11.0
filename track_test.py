from arucoTracking import PositionTracker
from utils import ARUCO_DICT, plot_track
import numpy as np
import time

start = time.time()
coords1 = []
coords2 = []
aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
calibration_matrix_path = "calibration_data/calibration_matrix.npy"
distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"
k_values =  [[1000 , 1000, 1000], 
             [ 0.1,  0.1, 0.1], 
             [  1000,   600,   600]] # PID, TPR

range = [[1300, 1700], [1300, 1700], [1300, 1700]] # TPR

k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

#........................................................

#initialize tracking and Drone communication.............

pos_tracker = PositionTracker(aruco_dict_type, k, d, camera_src=2, wait_time=1)
pos_tracker.start()

while time.time()-start<20:
    pos2 = pos_tracker.read_smooth_position(0)
    pos1 = pos_tracker.read_position(0)

    if(len(pos1) == 3 and len(pos2) == 3):
        coords1.append(pos1)
        coords2.append(pos2)
        print(pos1)

pos_tracker.stop()
plot_track(coords1, coords2, 10)
