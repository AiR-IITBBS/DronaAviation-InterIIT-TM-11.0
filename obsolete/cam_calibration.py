import numpy as np
import cv2
import time

cv2.namedWindow("Image Feed")
cv2.moveWindow("Image Feed", 159, -25)

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 40)

cal_img_count = 1
frame_count = 0

prev_frame_time = time.time()
while True:
    ret, frame = cap.read()
    frame_count += 1
    if frame_count == 30:
        cv2.imwrite(r"../test_calib/img ("+str(cal_img_count)+").jpg", frame)
        cal_img_count += 1
        frame_count = 0

    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    cv2.putText(frame, "FPS"+str(int(fps)), (10, 40),
                cv2.FONT_HERSHEY_PLAIN, 3, (100, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow("Image Feed", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
