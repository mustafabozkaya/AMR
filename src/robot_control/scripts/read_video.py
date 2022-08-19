import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import sys
from read_camera import *
import time

cap = cv2.VideoCapture("./line_video.mp4")
while cap.isOpened():
    ret, frame = cap.read()
    frames = []  # stores the video sequence for the demo
    # Video capture parameters
    (w, h) = (640, 240)  # Resolution
    bytesPerFrame = w * h
    fps = 40  # setting to 250 will request the maximum framerate possible

    lateral_search = 20  # number of pixels to search the line border
    start_height = h - 5  # Scan index row 235

    if ret == True:

        no_points_count = 0
        # set the correct dimensions for the numpy array for easier access to rows, now rows are columns
        frame = cv2.resize(frame, (w, h))
        start_time = time.time()
        # Drawing color points requires RGB image
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # ret, thresh = cv2.threshold(frame, 105, 255, cv2.THRESH_BINARY)
        tresh = cv2.adaptiveThreshold(
            frame_rgb, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        signed_thresh = tresh[start_height].astype(
            np.int16)  # select only one row
        # The derivative of the start_height line
        diff = np.diff(signed_thresh)
        # maximums and minimums of derivative
        points = np.where(np.logical_or(diff > 200, diff < -200))
        cv2.line(frame_rgb, (0, start_height), (640, start_height),
                 (0, 255, 0), 1)  # draw horizontal line where scanning
        # if finds something like a black line
        if len(points) > 0 and len(points[0]) > 1:

            middle = (points[0][0] + points[0][1]) / 2
            cv2.circle(frame_rgb, (points[0][0],
                       start_height), 2, (255, 0, 0), -1)
            cv2.circle(frame_rgb, (points[0][1],
                       start_height), 2, (255, 0, 0), -1)
            cv2.circle(frame_rgb, (middle, start_height), 2, (0, 0, 255), -1)
            print(int((middle-320)/int(sys.argv[1])))

        else:
            start_height -= 5
        start_height = start_height % h
        no_points_count += 1

        print("Loop took:", str((time.clock() - start_time) * 1000), 'ms')
        frames.append(frame_rgb)
        frames.append(thresh)

        cv2.imshow("frame", frame)
        cv2.imshow("thresh", thresh)
        cv2.imshow("frame_rgb", frame_rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break
cap.release()
cv2.destroyAllWindows()
