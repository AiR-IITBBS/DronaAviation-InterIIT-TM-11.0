from utils import ARUCO_DICT, print_coordinates , plot, plot_track
from arucoTracking import PositionTracker
from Communication import Drone
from pidaxischanged import PIDController
import time as tm
import numpy as np

def visit_checkpoints( checkpoints, flight_duration_per_checkpoint = 7 , x_permissible_error = 0.05 , y_permissible_error = 0.05, z_permissible_error = 0.05, id=0 ):

    # setup values..........................................
    
    commands_data=[]
    coords_data=[]

    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"
    k_values =  [[500 , 200, 200], 
                 [ 0.1,  0.1, 0.145], 
                 [ 6000,  6000 , 7000]]# PID, TPR

    range = [[1300, 1700], [1300, 1700], [1300, 1700]] # TPR

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    #........................................................

    #initialize tracking and Drone communication.............

    pos_tracker = PositionTracker(aruco_dict_type, k, d, camera_src=2, wait_time=1)
    pos_tracker.start()
    init_time = tm.time()

    drone = Drone("192.168.4.1", 23, 5, debug=True) #createss drone object, set debug to true to get console output on every action.
    drone.connect() #starts looping in a separate thread


    init_pos = [] 
    start = tm.time()
    while( len(init_pos)==0 ):
        print("Detecting Drone")
        init_pos = np.array(pos_tracker.read_smooth_position(id)) 
    
    tm.sleep(1)
    pos_tracker.set_origin(np.array(pos_tracker.read_smooth_position(id)))

    pid = PIDController([0, 0, -0.4], k_values, range)
    #[-0.31352251, -0.16215789,  0.8]
    drone.disarm()
    drone.takeoff()
    
    for i in checkpoints:
        pid.set_target(i)
        curr_err = pid.get_error()
        start = tm.time()
        while( (curr_err[0]>x_permissible_error or curr_err[1]>y_permissible_error or curr_err[2]>z_permissible_error) and tm.time()-start<flight_duration_per_checkpoint):

            new_pos = np.array(pos_tracker.read_smooth_position(id)) 
            print_coordinates(new_pos)
            if(len(new_pos) == 0):
                break
            # new_pos[0] *= 3.2
            # new_pos[1] *= 2
            # new_pos[2] *= 1.3
            calculated_state = pid.calculate_state(new_pos)
            commands_data.append(calculated_state)
            coords_data.append( new_pos )
            drone.set_state(calculated_state[0], calculated_state[1], calculated_state[2])
            tm.sleep(0.01)
            curr_err = pid.get_error()

    drone.land()
    pos_tracker.stop()
    drone.disconnect()
    plot(commands_data,coords_data,tm.time()-init_time,all=True)
    return

x_translate_checkpoints = [ [0,0,-0.4] , [0.5,0,-0.4] , [0.5,0,-0] ]
full_rect_check_points = [ [0,0,-0.4] , [0.5,0,-0.4] , [0.5,0.4,-0.4] , [0,0.4,-0.4] , [0,0,-0.4] , [0,0,0]]

visit_checkpoints(x_translate_checkpoints)