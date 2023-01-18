# import Drone as dr
import time as tm
import numpy as np
from arucoTracking import PositionTracker
from utils import ARUCO_DICT, print_coordinates
import pid_controller as PID

# drone = dr.Drone("192.168.4.1", 23, 1)

# drone.Kp = [100 , 100 , 100]
# drone.Ki =[0,0,0]
# # [250 , 250 , 125]
# drone.Kd = [0,0,0]
# [125 , 125 , 60]

# flight_time = 5

# checkpoint_count = 0

def hover_test(test_time = 10 , signal_delay = 0.035, id=5):

    aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    pos_tracker = PositionTracker(aruco_dict_type, k, d)
    pos_tracker.start()

    pid = PID.pid( [0,0,0.5] )      #the parameter is set-point

    init_pos = np.array(pos_tracker.read_position(id))   

    while( len(init_pos)==0 ):
        # print(init_pos)
        print("Detecting Drone")
        tm.sleep(1)
        init_pos = np.array(pos_tracker.read_position(id))  #pose-estimation

    print(init_pos)

    # print("Drone Detected")
    # drone.disarm()
    # drone.arm()
    # try:
    #     drone.disarm()
    #     drone.arm()
    #     drone.takeoff()
    # except:
    #     drone.disarm()
    #     drone.arm()
    #     drone.takeoff()
    # drone.checkpoint(0)
    start = tm.time()
    # print(PID.Drone_error[2] > PID.permissible_error_throttle)
    # while( tm.time()-start < test_time and PID.Drone_error[2] <= PID.permissible_error_throttle ):
    while( tm.time()-start < test_time):

        new_pos = np.array(pos_tracker.read_position(id))   

        if(len(new_pos) == 0):
            break

        pid.pid(new_pos)
        # drone.drone_position = np.array(new_pos) - init_pos    #pose-estimation
        # drone.pid()
        # drone.roll((dr.Drone_roll))
        # drone.pitch((dr.Drone_pitch)) 
        # drone.throttle((dr.Drone_throttle))
        # print(dr.Drone_pitch,dr.Drone_roll,dr.Drone_throttle)
        print_coordinates(new_pos)
        tm.sleep(signal_delay)

    # drone.land()
    # drone.disarm()
    # drone.disconnect()

hover_test()

# def rectangle_test(test_time = 5 , signal_delay = 0.035):

#     aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
#     calibration_matrix_path = "calibration_data/calibration_matrix.npy"
#     distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

#     k = np.load(calibration_matrix_path)
#     d = np.load(distortion_coefficients_path)

#     video = cv.VideoCapture(2)
#     tm.sleep(1.0)

#     ret, frame = video.read()

#     init_pos = np.array(pose_estimation(frame, aruco_dict_type, k, d))   #pose-estimation


#     drone.arm()
#     drone.takeoff()
#     drone.checkpoint(checkpoint_count)
#     start = tm.time()

#     while(tm.time() - start < test_time and (dr.Drone_error[0]>dr.permissible_error_location and dr.Drone_error[1]>dr.permissible_error_location)):

#         ret, frame = video.read()
#         drone.drone_position = np.array(pose_estimation(frame, aruco_dict_type, k, d)) - init_pos    #pose-estimation
#         drone.roll((dr.Drone_roll))
#         drone.pitch((dr.Drone_pitch)) 
#         drone.throttle((dr.Drone_throttle))
#         print(dr.Drone_pitch,dr.Drone_roll,dr.Drone_throttle)
#         tm.sleep(signal_delay)

#     drone.land()
#     drone.disarm()
#     drone.disconnect()
