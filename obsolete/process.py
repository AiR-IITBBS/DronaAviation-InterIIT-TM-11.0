import numpy as np
import cv2
import glob

# random values to  be measured and update at the time of calibration
cb_width = 11
cb_height = 8
cb_squaresize =2.4

# criterias for termination
criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cb_3d_points = np.zeros((cb_height*cb_width, 3), np.float32)
cb_3d_points[:, :2] = np.mgrid[0:cb_width,
                               0:cb_height].T.reshape(-1, 2)*cb_squaresize

list_cb_3d_points = []
list_cb_2d_img_points = []

list_imgs = glob.glob('*.jpg')

for frame_name in list_imgs:
    img = cv2.imread(frame_name)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

    if ret == True:
        list_cb_3d_points.append(cb_3d_points)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        list_cb_2d_img_points.append(corners2)

        cv2.drawChessboardCorners(img, (cb_width, cb_height), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)


cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    list_cb_3d_points, list_cb_2d_img_points, gray.shape[::-1], None, None)

print("Calibration Matrix: ")
print(mtx)
print("Distorsion", dist)

with open('camera_cal.npy', 'wb') as f:
    np.save(f, mtx)
    np.save(f, dist)
