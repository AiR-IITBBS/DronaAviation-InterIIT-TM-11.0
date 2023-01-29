from threading import Thread
from numpy import array as arr
import cv2
import time
import statistics
import numpy as np

parameters = cv2.aruco.DetectorParameters_create()


# parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
# parameters.minOtsuStdDev = 3.0
# parameters.maxErroneousBitsInBorderRate = 0.35
# parameters.perspectiveRemovePixelPerCell = 10
# parameters.perspectiveRemoveIgnoredMarginPerCell = 0.15
# parameters.maxErroneousBitsInBorderRate = 0.8
# parameters.errorCorrectionRate = 0.8 

# parameters.minMarkerPerimeterRate = 0.02
# # parameters.cornerRefinementMaxIterations = 70
# # parameters.cornerRefinementMinAccuracy = 0.2
       

class PositionTracker:
    def __init__(self, aruco_dict_type, matrix_coefficients, distortion_coefficients,wait_time=1, display=True, camera_src=0, smoothing=[5, 5, 10]):
        self.stream = cv2.VideoCapture(camera_src , cv2.CAP_DSHOW)
        time.sleep(2)
        # self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        # self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1280)

        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        # self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
        
        self.aruco_dict_type = aruco_dict_type
        cv2.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
        (self.grabbed, self.frame) = (False, [])
        self.stopped = False
        self.position = {}
        self.rotation = {}
        self.last_track_time = {}
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients
        self.display = display
        self.wait_time = wait_time
        self.origin_position = [0.0, 0.0, 0.0]
        self.smooth_position = {}
        self.position_store = {}
        self.smoothing = smoothing

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def read_position(self, id):
        return self.position.get(id,[])

    def read_z_rotation(self,id):
        return self.rotation.get(id,0)

    def read_smooth_position(self, id):
        return self.smooth_position.get(id, [])

    def read_frame(self):
        return self.frame

    def stop(self):
        self.stopped = True
        time.sleep(1) 
        self.stream.release()
        cv2.destroyAllWindows()
        
    def set_origin(self, origin):
        self.origin_position = origin
        
    def update(self):
        
        while True:
            clock = time.time()
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()
            if not self.grabbed:
                print('debug')
                self.stop()
                return
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
                cameraMatrix=self.matrix_coefficients,
                distCoeff=self.distortion_coefficients) 

            if len(corners) > 0:
                for i in range(0, len(ids)):
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.matrix_coefficients, self.distortion_coefficients)
                    cv2.aruco.drawDetectedMarkers(self.frame, corners) 
                    cv2.aruco.drawAxis(self.frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)
                    self.position[ids[i][0]] = list(arr(tvec[0][0]) - arr(self.origin_position))
                    self.rotation[ids[i][0]] = rvec[0][0][2]
                    self.generate_smooth_position(ids[i][0])
                    self.last_track_time[ids[i][0]] = time.time()
                    
                    
            for key in self.last_track_time:
                if((time.time() - self.last_track_time[key] > self.wait_time) and key in self.position):
                    self.position.pop(key)
                    self.smooth_position.pop(key)
                    self.position_store.pop(key)
            # print(time.time()-clock)
            if(self.display):
                cv2.imshow('Drone Tracking', cv2.resize(self.frame, (540, 360)))

            cv2.waitKey(1)

    def generate_smooth_position(self, id):
        if self.position_store.get(id, 0) == 0:
            self.position_store[id] = []
        self.position_store[id].insert(0, self.position[id]+[time.time()])
        if len(self.position_store[id]) > max(self.smoothing):
            self.position_store[id].pop()
        
        # for pos in self.position_store[id]:
        #     self.smooth_position[id][0] += (pos[0]/self.smoothing)
        #     self.smooth_position[id][1] += (pos[1]/self.smoothing)
        #     self.smooth_position[id][2] += (pos[2]/self.smoothing)
        self.smooth_position[id] = self.moving_average(self.position_store[id])
        
        # self.smooth_position[0] = statistics.median(np.array(self.position_store)[:,0])
        # self.smooth_position[1] = statistics.median(np.array(self.position_store)[:,1])
        # self.smooth_position[2] = statistics.median(np.array(self.position_store)[:,2])
    def moving_average(self, position_store):
        n = len(position_store)
        smooth_position = np.array([0.0, 0.0, 0.0])
        n1 = min(n, self.smoothing[0])
        for i in range(n1):
            smooth_position[0] += position_store[i][0]/n1
        n2 = min(n, self.smoothing[1])
        for i in range(n2):
            smooth_position[1] += position_store[i][1]/n2
        n3 = min(n, self.smoothing[2])
        for i in range(n3):
            smooth_position[2] += position_store[i][2]/n3
        # smooth_position[2] = statistics.median(np.array(position_store)[:,2])
        return list(smooth_position)

    def median_smoothing(self, position_store):
        position_store = np.array(position_store)
        n = len(position_store)
        x_mean = sum(position_store[:,0])/n
        y_mean = sum(position_store[:,1])/n
        z_median = statistics.median(position_store[:,2])
        return [x_mean,y_mean,z_median]


    def get_velocity(self,id=0):
        if(len(self.position_store[id])<=2 or self.position_store.get(id,0)==0):
            return 10000
        position = np.array(self.position_store.get(id))
        velocity = (position[0][0:3] - position[1][0:3])/(position[0][3]-position[1][3])
        rms_velo = velocity[0]**2 + velocity[1]**2 + velocity[2]**2
        return rms_velo





        
            
            

    
