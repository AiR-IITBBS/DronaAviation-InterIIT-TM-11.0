from arucoTracking import PositionTracker
from Communication import Drone
from utils import ARUCO_DICT, print_coordinates
import pid_controller as PID
import time as tm
import numpy as np
import matplotlib.pyplot as plt

def hover_test( flight_duration = 10 , id=5 ):

    throttle_data = []
    pitch_data = []
    roll_data = []
    z_coord = []
    x_coords = []
    y_coords = []

    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    pos_tracker = PositionTracker(aruco_dict_type, k, d, camera_src=0, wait_time=1)
    pos_tracker.start()

    drone = Drone("192.168.4.1", 23, 5, debug=True) #creates drone object, set debug to true to get console output on every action.
    drone.connect() #starts looping in a separate thread

    pid = PID.pid([-0.31352251, -0.16215789,  0.8])
    #[-0.31352251, -0.16215789,  0.8]

    init_pos = np.array(pos_tracker.read_position(id))   
    start = tm.time()
    while( len(init_pos)==0 and tm.time()-start < flight_duration):
        # print(init_pos)
        print("Detecting Drone")
        tm.sleep(1)
        init_pos = np.array(pos_tracker.read_position(id))  #pose-estimation

    print("Initial Position",init_pos)
    drone.takeoff()
    
    while( tm.time()-start < flight_duration):

        # new_pos = np.array(pos_tracker.read_position(id)) - init_pos  
        new_pos = np.array(pos_tracker.read_position(id))  
 
        print(new_pos)
        if(len(new_pos) == 0):
            break
        params = pid.pid(new_pos)
        throttle_data.append(params[0])
        # pitch_data.append(params[1])
        # roll_data.append(params[2])
        # x_coords.append(new_pos[0])
        # y_coords.append(new_pos[1])
        z_coord.append( new_pos[2] )

        drone.set_state(params[0], params[1], params[2])
    drone.land()
    pos_tracker.stop()
    drone.disconnect()
    print(len(throttle_data))
    print(len(pitch_data))

    x = np.linspace(0 , flight_duration , len(throttle_data))
    plt.scatter(x,throttle_data,s=1)
    # plt.scatter(x,pitch_data,s=1)
    # plt.scatter(x,roll_data,s=1)
    plt.scatter(x,np.array(z_coord)*100 + 1500,s=1)
    # plt.scatter(x,np.array(x_coords)*100 + 1500,s=1)
    # plt.scatter(x,np.array(x_coords)*100 + 1500,s=1)

    plt.show()
    return


hover_test()

