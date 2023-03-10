# This file is yet to be finalized
from arucoTracking import PositionTracker
from utils import ARUCO_DICT
from Communication import Drone
from pidaxischanged import PIDController
import time as tm
import numpy as np
from math import fabs
import threading 

def visitCheckpoints(checkpoints,drone,pos_tracker,pid,id):
    for i in checkpoints:
        pid.set_target(i[0])
        # curr_err = pid.get_error()
        start = tm.time()
        # while( curr_err[0]>x_permissible_error or curr_err[1]>y_permissible_error or curr_err[2]>z_permissible_error or pos_tracker.get_velocity(id)>permissible_rms_velocity):
        while(tm.time()-start<i[1]):
            new_pos = np.array(pos_tracker.read_smooth_position(id))
            # new_z_rot = pos_tracker.read_z_rotation(id)
            if(len(new_pos) == 0):
                break
            # print(new_pos)
            calculated_state = pid.calculate_state(new_pos)
            drone.set_state(calculated_state[0], calculated_state[1], calculated_state[2])
            tm.sleep(0.035)
            # curr_err = pid.get_error()
        # print("Curr_err",curr_err)
        # print("Checkpoint_reached")
    drone.land()
    tm.sleep(2.5)
    drone.disconnect()
    return

def nSwarm(checkpoints,ID,IPs):
    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"
    k_values =   [[250 , 160, 160],
                 [ 0.05,  0.1, 0.1],
                 [ 8500,  8500 , 8500]]# PID, TPRs

    range = [[1300, 2000], [1300, 1700], [1300, 1700]] # TPR

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)


    pos_tracker = PositionTracker(aruco_dict_type, k, d, camera_src=0, wait_time=1 ,  smoothing=[3, 3, 10])
    pos_tracker.start()
    pos_tracker.set_scaling_params([2.36, 2.4375, 2.56])
    init_pos = np.array(pos_tracker.read_smooth_position(ID[0]))
    # start = tm.time()
    while( len(init_pos)==0 ):
        print("Detecting Drone")
        init_pos = np.array(pos_tracker.read_smooth_position(ID[0]))
    
    print("Initial Position",init_pos)
    tm.sleep(1)
    print("Origin Set")
    pos_tracker.set_origin(np.array(pos_tracker.read_smooth_position(ID[0])))

    

    droneSet ={}
    for i in ID:
         drone = Drone(IPs[i], 23, i, debug=True) #createss drone object, set debug to true to get console output on every action.
         drone.connect()
         pid = PIDController([0, 0, 0], k_values, range)
         droneSet[i] = (drone , pid)
    droneThreads = {}

    for i in droneSet:
        droneObj = droneSet[i][0]
        pidObj = droneSet[i][1]

        droneObj.disarm()
        droneObj.takeoff()
        tm.sleep(0.2)

        checkPointThread = threading.Thread(target = visitCheckpoints , args = (checkpoints , droneObj , pos_tracker , pidObj , i) )

        droneThreads[i]= checkPointThread
        checkPointThread.start()

        tm.sleep(8)

    for i in droneThreads:
        droneThreads[i].join()
    
    pos_tracker.stop()
    return


if __name__ == "__main__":
    hover = [[[0,0,-0.4],12]]

    z=-0.3
    x_translate_checkpoints = [ [[0,0,z] , 10] , [[-0.5,0,z] , 7], [[-1,0,z] , 7]]


    nSwarm(x_translate_checkpoints , [3,0] , ["192.168.137.124" , "192.168.137.83"])











    


