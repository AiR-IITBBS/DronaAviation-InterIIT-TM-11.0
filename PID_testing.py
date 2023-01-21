# import Drone as dr
import time as tm
import numpy as np
from arucoTracking import PositionTracker
from utils import ARUCO_DICT, print_coordinates, plot
import pid_controller as PID

def hover_test(test_time = 10 , signal_delay = 0.022, id=5):

    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    pos_tracker = PositionTracker(aruco_dict_type, k, d)
    pos_tracker.start()

    
    init_pos = np.array(pos_tracker.read_position(id))   

    while( len(init_pos)==0 ):
        print("Detecting Drone")
        tm.sleep(1)
        init_pos = np.array(pos_tracker.read_position(id))  #pose-estimation

    print(init_pos)

    pid = PID.pid( [0,0,0],init_pos)      #the parameter is set-point


    start = tm.time()
    
    coords_data=[]
    commands_data=[]

    while( tm.time()-start < test_time):

        new_pos = np.array(pos_tracker.read_position(id))
        coords_data.append(new_pos)   
        print_coordinates(new_pos)
        if(len(new_pos) == 0):
            break
        
        curr_command = pid.pid(new_pos)
        commands_data.append(curr_command)
        tm.sleep(signal_delay)

    pos_tracker.stop()
    print("Origin",init_pos*(-1))
    plot(commands_data,coords_data,test_time,y="pitch",all=False)

hover_test(7)
