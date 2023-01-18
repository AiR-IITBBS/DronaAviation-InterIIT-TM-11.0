from arucoTracking import PositionTracker
from utils import ARUCO_DICT
import numpy as np
import time
import cv2
from threading import Thread

aruco_dict_type = ARUCO_DICT["DICT_4X4_250"]
calibration_matrix_path = "calibration_data/calibration_matrix.npy"
distortion_coefficients_path = "calibration_data/distortion_coefficients.npy"

k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

pos_tracker = PositionTracker(aruco_dict_type, k, d)
pos_tracker.start()

start = time.time()
while (time.time()-start < 20):
    print(pos_tracker.read_position(10))
    time.sleep(0.5)
pos_tracker.stop()




