from utils import ARUCO_DICT, print_coordinates , plot
from arucoTracking import PositionTracker
from Communication import Drone
from pidaxischanged import PIDController
import time as tm
import numpy as np
import matplotlib.pyplot as plt

def hover(flight_duration = 25 , id=0 ):

    # setup values..........................................
    
    commands_data=[]
    coords_data=[]

    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"
    k_values =  [[400 , 150, 150], 
                 [ 0.15,  0.2, 0.15], 
                 [ 8000,  7500 , 7000]]# PID, TPR

                #  [[300 , 200, 200], 
                #  [ 0.000001,  0.000001, 0.000001], 
                #  [ 6000,  7000 , 7000]]

    range = [[1300, 1700], [1300, 1700], [1300, 1700]] # TPR

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    #........................................................

    #initialize tracking and Drone communication.............

    pos_tracker = PositionTracker(aruco_dict_type, k, d, camera_src=2, wait_time=1, smoothing=[5, 5, 20])
    pos_tracker.start()

    drone = Drone("192.168.4.1", 23, 5, debug=True) #createss drone object, set debug to true to get console output on every action.
    drone.connect() #starts looping in a separate thread


    init_pos = np.array(pos_tracker.read_smooth_position(id))   
    
    while( len(init_pos)==0):
        print("Detecting Drone")
        init_pos = np.array(pos_tracker.read_smooth_position(id)) 
    
    tm.sleep(1)
    pos_tracker.set_origin(np.array(pos_tracker.read_smooth_position(id)))

    pid = PIDController([0, 0, -0.4], k_values, range)
    #[-0.31352251, -0.16215789,  0.8]

    print("Initial Position",init_pos)
    drone.disarm()
    drone.takeoff()
    tm.sleep(0.3)

    start = tm.time()
    
    while( tm.time()-start < flight_duration):

        # new_pos = np.array(pos_tracker.read_position(id)) - init_pos  
        new_pos = np.array(pos_tracker.read_smooth_position(id))
 
        # print(new_pos)
        if(len(new_pos) == 0):
            print("Couldn't detect")
            break
        new_pos[0] *= 2.2      #2.42,2.0,2.75
        new_pos[1] *= 2.2
        new_pos[2] *= 2.2
        calculated_state = pid.calculate_state(new_pos)
        commands_data.append(calculated_state)
        coords_data.append( new_pos )
        drone.set_state(calculated_state[0], calculated_state[1], calculated_state[2])
        tm.sleep(0.02)

    pid.set_target([-2, 0, -0.4])
    start = tm.time()
    print('Checkpoint Reached')

    while( tm.time()-start < flight_duration):

        # new_pos = np.array(pos_tracker.read_position(id)) - init_pos  
        new_pos = np.array(pos_tracker.read_smooth_position(id))
        if(len(new_pos) == 0):
            print("Couldn't detect")
            break
        new_pos[0] *= 2.2      #2.42,2.0,2.75
        new_pos[1] *= 2.2
        new_pos[2] *= 2.2
        calculated_state = pid.calculate_state(new_pos)
        commands_data.append(calculated_state)
        coords_data.append( new_pos )
        drone.set_state(calculated_state[0], calculated_state[1], calculated_state[2])
        tm.sleep(0.02)
    drone.land()
    pos_tracker.stop()
    drone.disconnect()
    print(pid.get_error())
    plot(commands_data,coords_data,flight_duration,all=True)
    return


hover(5)

