from utils import ARUCO_DICT, print_coordinates , plot, plot_velo
from arucoTracking import PositionTracker
from Communication import Drone
from pidaxischanged import PIDController
import time as tm
import numpy as np
from math import fabs

def visit_checkpoints( checkpoints, x_permissible_error = 0.07 , y_permissible_error = 0.07, z_permissible_error = 0.05 , permissible_rms_velocity=0.05, id=0 ):
#checkpoints format: [ [ [x,y,z] , time_alloted ] , ... ]

    # setup values..........................................
    
    commands_data=[]
    coords_data=[]
    velocity_arr = []
    z_rot = []

    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

    k_values =   [[250 , 160, 160],
                 [ 0.05,  0.1, 0.1], 
                 [ 8500,  8500 , 8500]]     # PID, TPRs

    range = [[1300, 1900], [1300, 1700], [1300, 1700]] # TPR    #maximum values for state

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    #........................................................

    #initialize tracking and Drone communication.............

    pos_tracker = PositionTracker(aruco_dict_type, k, d, camera_src=2, wait_time=1 ,  smoothing=[3, 3, 10])
    pos_tracker.start()

    drone = Drone("192.168.4.1", 23, 5, debug=True) #creates drone object, set debug to true to get console output on every action.
    drone.connect() #starts looping in a separate thread

    init_pos = np.array(pos_tracker.read_smooth_position(id))
    while( len(init_pos)==0 ):
        print("Detecting Drone")
        init_pos = np.array(pos_tracker.read_smooth_position(id))
    
    print("Initial Position",init_pos)
    tm.sleep(1)
    print("Origin Set")
    pos_tracker.set_origin(np.array(pos_tracker.read_smooth_position(id)))      #sets the initial position as origin
    tm.sleep(2)
    pid = PIDController([0, 0, -0.4], k_values, range)
    pid.calculate_state(np.array(pos_tracker.read_smooth_position(id)))
    
    init_time = tm.time()
    drone.disarm()
    drone.takeoff()
    tm.sleep(0.2)
    
    for i in checkpoints:
        pid.set_target(i[0])
        curr_err = pid.get_error()
        start = tm.time()
        # while( curr_err[0]>x_permissible_error or curr_err[1]>y_permissible_error or curr_err[2]>z_permissible_error or pos_tracker.get_velocity(id)>permissible_rms_velocity):
        while(tm.time()-start<i[1]):
            new_pos = np.array(pos_tracker.read_smooth_position(id))    #getting latest position
            new_z_rot = pos_tracker.read_z_rotation(id)                 #getting current yaw state

            if(len(new_pos) == 0):                                      #terminates the current loop if tracking fails for more than the wait_time(defined earlier)
                break

            #scaling obtained values
            new_pos[0] *= 2.36                      
            new_pos[1] *= 2.4375
            new_pos[2] = -fabs(new_pos[2])*2.56
            #....................................

            # print(new_pos)
            calculated_state = pid.calculate_state(new_pos)     #

            #storing data for plotting
            commands_data.append(calculated_state)
            coords_data.append( new_pos )
            velocity_arr.append(pos_tracker.get_velocity(id))
            z_rot.append(fabs(new_z_rot))
            #...................................................

            drone.set_state(calculated_state[0], calculated_state[1], calculated_state[2])      #setting the new state values
            tm.sleep(0.03)
            curr_err = pid.get_error()

        print("Final Error",curr_err)
        print("Checkpoint_reached")

    
    pos_tracker.stop()
    drone.land()
    drone.disconnect()

    total_flight_duration = tm.time()-init_time
    plot(commands_data,coords_data,z_rot,total_flight_duration,all=True)        #plots the recorded data
    # plot_velo(velocity_arr,total_flight_duration)
    return

testing = [[[0,0,-0.5],30]]  

hover = [[[0,0,-0.4],12]]           #checkpoint for hover test

x = -2
y = 1
z = -0.4
cp_time = 6     #checkpoint time
hover_time = 8

rectangle = [  [[0,0,z] , hover_time] ,                                                                    #checkpoints for rectangle test
 [[x/4,0,z] , cp_time], [[x/2,0,z] , cp_time], [[x*3/4,0,z] , cp_time],  [[x,0,z] , cp_time], 
 [[x,y/3,z] ,cp_time], [[x,y*2/3,z] , cp_time] ,  [[x,y,z] , cp_time] ,
 [[x*3/4,y,z] , cp_time], [[x/2,y,z] , cp_time], [[x/4,y,z] , cp_time],  [[0,y,z] , cp_time],
 [ [0,y*2/3,z],cp_time] , [[0,y/3,z],cp_time], [[0,0,z],cp_time]  ]

x_translate_checkpoints = [ [[0,0,z] , 13] , [[-0.5,0,z] , 10], [[-1,0,z] , 10], [[-1.5,0,z] , 10], [[-2,0,z] , 10]]
x_y_translate = [ [[0,0,z] , 13] , [[-0.5,0,z] , 10], [[-1,0,z] , 10], [[-1,0.4,z] , 10], [[-1,0.8,z] , 10] ]

# visit_checkpoints(x_translate_checkpoints)
visit_checkpoints(rectangle)