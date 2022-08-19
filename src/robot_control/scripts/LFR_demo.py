import cv2
import numpy as np
# 0 -> index of camera to use (i.e. 0 -> /dev/video0)
# cap = cv2.VideoCapture("/dev/video0")

cap = cv2.VideoCapture("./line_video.mp4")

cap.set(3, 320)
cap.set(4, 240)


while cap.isOpened():
    ret, frame = cap.read()

    if ret:

        low_b = np.uint8([0, 0, 130])
        high_b = np.uint8([179, 255, 255])
        converted = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # convert to HSV

        mask = cv2.inRange(converted, low_b, high_b)  # create mask

        contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
           
            cv2.drawContours(frame, c, -1, (0, 255, 0), 1)
            M = cv2.moments(c)
            if M["m00"] != 0 and M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("CX : "+str(cx)+"  CY : "+str(cy))
                if cx >= 120:
                    print("Turn Left")

                if cx < 120 and cx > 40:
                    print("On Track!")

                if cx <= 40:
                    print("Turn Right")

                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
        else:
            print("I don't see the line")

        cv2.imshow("converted", converted)
        cv2.imshow("Mask", mask)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms

            break
    else:
        break
cap.release()
cv2.destroyAllWindows()
