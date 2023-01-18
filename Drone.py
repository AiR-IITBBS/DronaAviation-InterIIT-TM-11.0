import socket as skt
import time as tm
import argparse
import sys
import numpy as np
import cv2 as cv
from utils import ARUCO_DICT, show_fps, print_coordinates
from arucoTracking import pose_estimation

Drone_roll=0.0
Drone_pitch=0.0
Drone_throttle=0.0

class Drone:
   

    def __init__(self, IP_ADDRESS, PORT, id):
        self.id = id
        self.sz = 1024
        self.com = skt.socket(skt.AF_INET, skt.SOCK_STREAM)
        self.com.connect((IP_ADDRESS, PORT))
        print("Pluto connected...")
        self.com.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 0)
        self.com.setsockopt(skt.IPPROTO_TCP, skt.TCP_KEEPCNT, 4)
        self.rc_raw_data = bytearray([36,77,60,16,200,220,5,220,5,220,5,220,5,176,4,232,3,220,5,176,4,234]) #header included
        self.set_cmd_data = bytearray([36,77,60,2,217,1,0,218]) #header included

        #For PID******************************************
        self.drone_position = [0.0,0.0,0.0]
        self.setpoint = [0,0,0]

        self.Kp = [0,0,0]
        self.Ki = [0,0,0]
        self.Kd = [0,0,0]

        self.error = [0.0,0.0,0.0]
        self.prev_value = [0.0,0.0,0.0]
        self.sum_error = [0.0,0.0,0.0]
        self.out_pitch = 0.0
        self.out_roll = 0.0
        self.out_throttle = 0.0
        self.max_values = [2000,2000,2000]
        self.min_values = [1000,1000,1000]

        self.drone_roll = 1500
        self.drone_pitch = 1500
        self.drone_throttle = 1500
        #**************************************************

    def update_checksum(self):
        checksum = 0
        for j in self.rc_raw_data[3:-1]:
            checksum ^= j
        self.rc_raw_data[21] = checksum
        checksum = 0
        for j in self.set_cmd_data[3:-1]:
            checksum ^= j
        self.set_cmd_data[7] = checksum

    def get_LSB_MSB(self, val):
        return bytearray([val % 256, val // 256])

    def arm(self):
        self.rc_raw_data[19] = 220
        self.rc_raw_data[20] = 5
        self.update_checksum()
        print("Pluto Armed...")
        self.com.send(self.rc_raw_data)
        tm.sleep(2)  # THIS WAS THE DAMN PROBLEM...absolutely need sleep after arm and disarm

    def disarm(self):
        self.rc_raw_data[19] = 176
        self.rc_raw_data[20] = 4
        self.update_checksum()
        print("Pluto Disarmed...")
        self.com.send(self.rc_raw_data)
        tm.sleep(2)   # THIS WAS THE DAMN PROBLEM...absolutely need sleep after arm and disarm

    def takeoff(self):
        self.set_cmd_data[5] = 1
        self.set_cmd_data[6] = 0
        self.update_checksum()
        self.com.send(self.set_cmd_data)

    def land(self):
        code_data = self.get_LSB_MSB(2)
        self.set_cmd_data[5] = code_data[0]
        self.set_cmd_data[6] = code_data[1]
        self.update_checksum()
        self.com.send(self.set_cmd_data)

    def throttle(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[9] = data[0]
        self.rc_raw_data[10] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def pitch(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[7] = data[0]
        self.rc_raw_data[8] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def roll(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[5] = data[0]
        self.rc_raw_data[6] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def yaw(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[11] = data[0]
        self.rc_raw_data[12] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def disconnect(self):
        self.com.close()
        tm.sleep(1)
        print('Pluto disconnected')

    #Yet to be tested [critical]
    def pid(self):
        global Drone_roll, Drone_pitch, Drone_throttle
        self.error[0] = self.drone_position[0] - self.setpoint[0]
        self.error[1] = self.drone_position[1] - self.setpoint[1]
        self.error[2] = self.drone_position[2] - self.setpoint[2]

        self.out_roll = int(self.Kp[0] * self.error[0] + self.Kd[0]*(self.error[0]-self.prev_value[0]) + self.Ki[0]*self.sum_error[0])
        self.out_pitch = int(self.Kp[1] * self.error[1] + self.Kd[1]*(self.error[1]-self.prev_value[1]) + self.Ki[1]*self.sum_error[1])
        self.out_throttle = int(self.Kp[2] * self.error[2] + self.Kd[2]*(self.error[2]-self.prev_value[2]) + self.Ki[2]*self.sum_error[2])

        self.drone_roll = 1500 - self.out_roll
        self.drone_pitch = 1500 + self.out_pitch
        self.drone_throttle = 1500 + self.out_throttle

        if(self.drone_roll > self.max_values[0] ):
            self.drone_roll = self.max_values[0]
        if(self.drone_pitch > self.max_values[1] ):
            self.drone_pitch = self.max_values[1]
        if(self.drone_throttle > self.max_values[2] ):
            self.drone_throttle = self.max_values[2]
        if( self.drone_roll < self.min_values[0]):
            self.drone_roll =  self.min_values[0]
        if( self.drone_pitch < self.min_values[1]):
            self.drone_pitch =  self.min_values[1]
        if( self.drone_throttle < self.min_values[2]):
            self.drone_throttle =  self.min_values[2]

        self.prev_value[0] = self.error[0]
        self.prev_value[1] = self.error[1]
        self.prev_value[2] = self.error[2]
        self.sum_error[0] = self.sum_error[0] + self.error[0]
        self.sum_error[1] = self.sum_error[1] + self.error[1]
        self.sum_error[2] = self.sum_error[2] + self.error[2]
        # Need to look into anti windup for limiting integral term.

        Drone_roll=self.drone_roll
        Drone_pitch=self.drone_pitch
        Drone_throttle=self.drone_throttle

def throttle_test(time):
    drone = Drone("192.168.4.1", 23, 1)
    # drone.disarm() 
    drone.arm()
    drone.takeoff()
    clock_start = tm.time()
    while(tm.time()-clock_start < time):
        drone.roll((Drone_roll)) 
        drone.pitch((Drone_pitch)) 
        drone.throttle((Drone_throttle)) 
        tm.sleep(0.022)
    drone.land()
    drone.disarm()
    drone.disconnect()

# try:
#   throttle_test(5)
# except ConnectionAbortedError:
#   throttle_test(5)

#--------------------------------main code---------------------------------
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = "calibration_data/calibration_matrix.npy"
    distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv.VideoCapture(2)
    tm.sleep(1.0)

    while True:
        ret, frame = video.read()
        if not ret:
            break
        output, tvec = pose_estimation(frame, aruco_dict_type, k, d)
        show_fps(output)
        cv.imshow('Estimated Pose', output)
        print_coordinates(tvec)
        #check for exit
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv.destroyAllWindows()
