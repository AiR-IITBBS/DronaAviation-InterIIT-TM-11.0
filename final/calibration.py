'''
Sample Usage:-
python calibration.py --dir calibration_checkerboard/ --square_size 0.024
'''

import numpy as np
import cv2
import time


def calibrate(cam_src, frame_count, square_size, width, height, visualize=False):
    """ Apply camera calibration operation for images in the given directory path. """

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    video_stream = cv2.VideoCapture(cam_src)
    video_stream.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    video_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
    time.sleep(1)

    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    curr_count = 0

    while True:
        if curr_count >= frame_count: 
            break
        (check, frame) = video_stream.read()
        print(frame.shape)
        if not check:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            frame = cv2.drawChessboardCorners(frame, (width, height), corners2, ret)

        if visualize:
            cv2.imshow('img',cv2.resize(frame,(540,360)))
            # cv2.imshow('img',frame)
            cv2.waitKey(0)
        
        curr_count += 1

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]


# def charuco_calibrate(rows, cols, square_size, marker_size, count=30, cam_src=2):
        
#     # ChAruco board variables
#     CHARUCOBOARD_ROWCOUNT = rows
#     CHARUCOBOARD_COLCOUNT = cols
#     ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

#     # Create constants to be passed into OpenCV and Aruco methods
#     CHARUCO_BOARD = cv2.aruco.CharucoBoard_create(
#             squaresX=CHARUCOBOARD_COLCOUNT,
#             squaresY=CHARUCOBOARD_ROWCOUNT,
#             squareLength=square_size,
#             markerLength=marker_size,
#             dictionary=ARUCO_DICT)

#     # Create the arrays and variables we'll use to store info like corners and IDs from images processed
#     corners_all = [] # Corners discovered in all images processed
#     ids_all = [] # Aruco ids corresponding to corners discovered
#     image_size = None # Determined at runtime


#     # This requires a set of images or a video taken with the camera you want to calibrate
#     # I'm using a set of images taken with the camera with the naming convention:
#     # 'camera-pic-of-charucoboard-<NUMBER>.jpg'
#     # All images used should be the same size, which if taken with the same camera shouldn't be a problem
#     video_stream = cv2.VideoCapture(cam_src)
#     video_stream.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
#     video_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1280)
#     time.sleep(2)

#     curr_count = 1
#     while curr_count < count:
#         # Open the image
#         (check, frame) = video_stream.read()
#         if not check:
#             continue
#         # Grayscale the image
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Find aruco markers in the query image
#         corners, ids, _ = cv2.aruco.detectMarkers(
#                 image=gray,
#                 dictionary=ARUCO_DICT)

#         # Outline the aruco markers found in our query image
#         frame = cv2.aruco.drawDetectedMarkers(
#                 image=frame, 
#                 corners=corners)

#         # Get charuco corners and ids from detected aruco markers
#         response, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
#                 markerCorners=corners,
#                 markerIds=ids,
#                 image=gray,
#                 board=CHARUCO_BOARD)

#         # If a Charuco board was found, let's collect image/corner points
#         # Requiring at least 20 squares
#         if response > 20:
#             # Add these corners and ids to our calibration arrays
#             corners_all.append(charuco_corners)
#             ids_all.append(charuco_ids)
            
#             # Draw the Charuco board we've detected to show our calibrator the board was properly detected
#             frame = cv2.aruco.drawDetectedCornersCharuco(
#                     image=frame,
#                     charucoCorners=charuco_corners,
#                     charucoIds=charuco_ids)
        
#             # If our image size is unknown, set it now
#             if not image_size:
#                 image_size = gray.shape[::-1]
        
#             # Reproportion the image, maxing width or height at 1000
#         else:
#             print("Not able to detect a charuco board in the image")

#     # Destroy any open CV windows

#         cv2.imshow('img',cv2.resize(frame,(1280,960)))
#         cv2.waitKey(0)
    
#     # Now that we've seen all of our images, perform the camera calibration
#     # based on the set of points we've discovered
#     calibration, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
#             charucoCorners=charuco_corners,
#             charucoIds=ids_all,
#             board=CHARUCO_BOARD,
#             imageSize=image_size,
#             cameraMatrix=None,
#             distCoeffs=None)
        
#     # Print matrix and distortion coefficient to the console
#     print(cameraMatrix)
#     print(distCoeffs)
        
#     # Save values to be used where matrix+dist is required, for instance for posture estimation
#     # I save files in a pickle file, but you can use yaml or whatever works for you
        
#     # Print to console our success
#     print('Calibration successful.')
#     np.save("calibration_data/calibration_matrix", cameraMatrix)
#     np.save("calibration_data/distortion_coefficients", distCoeffs)


if __name__ == '__main__':
    # time.sleep()
    ret, mtx, dist, rvecs, tvecs = calibrate(2, 40, 0.0178, 10, 7, True)

    print(mtx)
    print(dist)

    np.save("calibration_matrix", mtx)
    np.save("distortion_coefficients", dist)

    # charuco_calibrate(rows=8 , cols = 10 , square_size = 0.0178 , marker_size = 0.0160 , count = 35 , cam_src = 2 )

