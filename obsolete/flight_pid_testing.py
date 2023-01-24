from arucoTracking import PositionTracker
from Communication import Drone
from utils import ARUCO_DICT, print_coordinates , plot
import pid_controller as PID
import time as tm
import numpy as np
import matplotlib.pyplot as plt

def hover_test( flight_duration = 10 , id=5 ):

    commands_data=[]
    coords_data=[]

    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    pos_tracker = PositionTracker(aruco_dict_type, k, d, camera_src=2, wait_time=1)
    pos_tracker.start()

    drone = Drone("192.168.4.1", 23, 5, debug=True) #creates drone object, set debug to true to get console output on every action.
    drone.connect() #starts looping in a separate thread


    init_pos = np.array(pos_tracker.read_position(id))   
    start = tm.time()
    while( len(init_pos)==0 and tm.time()-start < flight_duration):
        # print(init_pos)
        print("Detecting Drone")
        tm.sleep(1)
        init_pos = np.array(pos_tracker.read_position(id))  #pose-estimation


    pid = PID.pid([-0.31352251, -0.16215789,  0.8] , init_pos)
    #[-0.31352251, -0.16215789,  0.8]

    print("Initial Position",init_pos)
    drone.takeoff()
    
    while( tm.time()-start < flight_duration):

        # new_pos = np.array(pos_tracker.read_position(id)) - init_pos  
        new_pos = np.array(pos_tracker.read_position(id))  
 
        print(new_pos)
        if(len(new_pos) == 0):
            break
        params = pid.pid(new_pos)
        commands_data.append(params)
        coords_data.append( new_pos )
        drone.set_state(params[0], params[1], params[2])
    drone.land()
    pos_tracker.stop()
    drone.disconnect()
    plot(commands_data,coords_data,flight_duration,all=True)
    return


hover_test()

