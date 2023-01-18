from threading import Thread
import cv2
import time

class PositionTracker:
    def __init__(self, aruco_dict_type, matrix_coefficients, distortion_coefficients,wait_time=1, display=True, camera_src=0):
        self.stream = cv2.VideoCapture(camera_src)
        time.sleep(1)
        (self.grabbed, self.frame) = (False, [])
        self.stopped = False
        self.position = {}
        self.last_track_time = {}
        self.aruco_dict_type = aruco_dict_type
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients
        self.display = display
        self.wait_time = wait_time

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def read_position(self, id):
        return self.position[id]

    def read_frame(self):
        return self.frame

    def stop(self):
        self.stopped = True
        time.sleep(1) 
        self.stream.release()
        cv2.destroyAllWindows()
        

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()
            if not self.grabbed:
                print('debug')
                self.stop()
                return
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            cv2.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
            parameters = cv2.aruco.DetectorParameters_create()

            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
                cameraMatrix=self.matrix_coefficients,
                distCoeff=self.distortion_coefficients)

            if len(corners) > 0:
                for i in range(0, len(ids)):
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.matrix_coefficients, self.distortion_coefficients)
                    cv2.aruco.drawDetectedMarkers(self.frame, corners) 
                    cv2.aruco.drawAxis(self.frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)
                    
                    self.position[ids[i][0]] = tvec[0][0]
                    self.last_track_time[ids[i][0]] = time.time()
            for key in self.last_track_time:
                if((time.time() - self.last_track_time[key] > self.wait_time) and key in self.position):
                    self.position.pop(key)
            if(self.display):
                cv2.imshow('Drone Tracking', self.frame)
            cv2.waitKey(1)
            

    
