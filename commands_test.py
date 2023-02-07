from Communication import Drone
# from utils import ARUCO_DICT, print_coordinates
# import pid_controller as PID
import time as tm
import numpy as np
import matplotlib.pyplot as plt
import socket as skt

# aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
# calibration_matrix_path = "calibration_data/calibration_matrix.npy"
# distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

# k = np.load(calibration_matrix_path)
# d = np.load(distortion_coefficients_path)

drone1 = Drone("192.168.137.83", 23, 5, debug=True) #creates drone object, set debug to true to get console output on every action.
drone1.connect() #starts looping in a separate thread
tm.sleep(2)

drone2 = Drone("192.168.137.124", 23, 5, debug=True) #creates drone object, set debug to true to get console output on every action.
drone2.connect() #starts looping in a separate thread
tm.sleep(2)

drone1.takeoff()
tm.sleep(2)
drone2.takeoff()
tm.sleep(2)

# client_socket = skt.socket(skt.AF_INET, skt.SOCK_STREAM)
# addr = ("192.168.137.36" , 139)
# client_socket.connect(addr)

# flight_duration = 4

# start = tm.time()

# while(tm.time()-start<flight_duration):
#     drone.set_state( 1500 , 1500 , 1100 )
#     tm.sleep(0.022)

drone1.land()
drone1.disconnect()

drone2.land()
drone2.disconnect()

# server = skt.socket(skt.AF_INET, skt.SOCK_STREAM , skt.IPPROTO_TCP)
# server.connect(("192.168.137.125" , 23))
# # server.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 0)
# # server.setsockopt(skt.IPPROTO_TCP, skt.TCP_KEEPCNT, 4)

# takeoff_arr = [36,77,60,2,217,1,0,218]
# state_arr = [36,77,60,16,200,220,5,220,5,220,5,220,5,176,4,232,3,220,5,176,4,234]

# server.sendall(bytearray(takeoff_arr))
# response = server.recv(1024)
# print(str(response))