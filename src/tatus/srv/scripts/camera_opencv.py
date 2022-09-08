#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class RobotKamera():
    def __init__(self):
        rospy.init_node("kamera")
        rospy.Subscriber("camera/rgb/image_raw", Image, self.cameraCallback)
        self.bridge = CvBridge()
        self.width = 480
        self.height = 400
        self.foto = np.uint8([[[0, 0, 0]]])
        self.CreateTrackBar_Init()
        rospy.spin()

    def cameraCallback(self, mesaj):
        self.foto = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        self.foto = cv2.resize(self.foto, (self.width, self.height))

        self.get_hsv_img()

    def get_camera_image(self):
        cv2.namedWindow("window", cv2.WINDOW_NORMAL)
        # set q of window
        cv2.resizeWindow("window", self.width, self.height)
        cv2.imshow("window", self.foto)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            print("Programdan çıkılıyor...")

    def get_hsv_img(self):

        frame = cv2.resize(self.foto, (640, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        Lower_H_Value = cv2.getTrackbarPos("Lower - H", "Trackbars")
        Lower_S_Value = cv2.getTrackbarPos("Lower - S", "Trackbars")
        Lower_V_Value = cv2.getTrackbarPos("Lower - V", "Trackbars")
        Upper_H_Value = cv2.getTrackbarPos("Upper - H", "Trackbars")
        Upper_S_Value = cv2.getTrackbarPos("Upper - S", "Trackbars")
        Upper_V_Value = cv2.getTrackbarPos("Upper - V", "Trackbars")

        lower = np.array([Lower_H_Value, Lower_S_Value, Lower_V_Value])
        upper = np.array([Upper_H_Value, Upper_S_Value, Upper_V_Value])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("ORİGİNAL İMG", frame)
        cv2.imshow("MASK", mask)
        cv2.imshow("RESULT", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            print("Programdan çıkılıyor...")

    def CreateTrackBar_Init(self):
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("Lower - H", "Trackbars", 0,
                           179, self.nothing)  # hue 0-179
        cv2.createTrackbar("Lower - S", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("Lower - V", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("Upper - H", "Trackbars", 169, 179, self.nothing)
        cv2.createTrackbar("Upper - S", "Trackbars", 204, 255, self.nothing)
        cv2.createTrackbar("Upper - V", "Trackbars", 204, 255, self.nothing)

    def nothing(self, x):
        pass


if __name__ == '__main__':
    nesne = RobotKamera()
