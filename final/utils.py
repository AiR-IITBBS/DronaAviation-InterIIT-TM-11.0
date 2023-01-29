import cv2
import time as tm
import matplotlib.pyplot as plt
import numpy as np

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def aruco_display(corners, ids, rejected, image):
	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()
		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned in
			# top-left, top-right, bottom-right, and bottom-left order)
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			# compute and draw the center (x, y)-coordinates of the ArUco
			# marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			# draw the ArUco marker ID on the image
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			print("[Inference] ArUco marker ID: {}".format(markerID))
			# show the output image
	return image

prev_frame_time = 0
def show_fps(output):
    global prev_frame_time
    new_frame_time = tm.time()
    fps = 1/(new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    fps = str(int(fps))
    cv2.putText(output, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)

def print_coordinates(tvec):
    if len(tvec) != 0:
        xpos = int(tvec[0]*100)
        ypos = int(tvec[1]*100)
        zpos = int(tvec[2]*100)
        print(f"x = {xpos} y = {ypos} z = {zpos}")

def plot(commands,coords,z_rot_data,flight_duration,y="pitch",all=False):
    commands=np.array(commands)
    coords=np.array(coords)
    coords=coords*100 + 1500
    x = np.linspace(0 , flight_duration , len(coords))
    # print(len(x) , len(coords[:,0]))    

    if(all):
        plt.figure(100),plt.subplot(1,4,1)
        plt.scatter(x , coords[:,0] , label='x-coords (scaled)' , s=2)
        plt.scatter(x , commands[:,1] , label='pitch', s=2)
        plt.xlabel('Time')
        plt.legend()

        plt.subplot(1,4,2)
        plt.scatter(x , coords[:,1] , label='y-coords (scaled)', s=2)
        plt.scatter(x , commands[:,2] , label='roll', s=2)
        plt.xlabel('Time')
        plt.legend()

        plt.subplot(1,4,3)
        plt.scatter(x , coords[:,2] , label='z-coords (scaled)', s=2)
        plt.scatter(x , commands[:,0] , label='throttle', s=2)
        plt.xlabel('Time')
        plt.legend()

        plt.subplot(1,4,4)
        plt.scatter(x , z_rot_data , label='z_rotation (scaled)', s=2)


        plt.show()
        return
    if(y=="throttle"):
        plt.scatter(x , coords[:,2] , label='z-coords (scaled)', s=2)
        plt.scatter(x , commands[:,0] , label='throttle', s=2)
        plt.xlabel('Time')
        plt.legend()
        plt.show()
    elif(y=="pitch"):
        plt.scatter(x , coords[:,0] , label='x-coords (scaled)', s=2)
        plt.scatter(x , commands[:,1] , label='pitch', s=2)
        plt.xlabel('Time')
        plt.legend()
        plt.show()
    elif(y=="roll"):
        plt.scatter(x , coords[:,1] , label='y-coords (scaled)', s=2)
        plt.scatter(x , commands[:,2] , label='roll', s=2)
        plt.xlabel('Time')
        plt.legend()
        plt.legend()
        plt.show()
    else:
        print("Invalid y-value")
    return

def plot_velo(velo_arr,flight_duration):
    velo_arr = np.array(velo_arr)*100
    x = np.linspace(0 , flight_duration , len(velo_arr))
    plt.figure(200),plt.scatter(x , velo_arr, s=2)
    plt.xlabel('Time')
    plt.ylabel('Velocity(cm/s)')
    plt.show()
    return


