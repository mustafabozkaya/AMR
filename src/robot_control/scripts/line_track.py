#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
line tracking
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class LineTrack():
    def __init__(self):
        rospy.init_node("track_line")
        self.bridge = CvBridge()
        self.cmd_vel = Twist()
        camera_topic = input(
            "Enter camera topic: (default: /camera/rgb/image_raw)")
        if camera_topic == "":
            self.camera_topic = "/camera/rgb/image_raw"
            rospy.Subscriber("/camera/rgb/image_raw",
                             Image, self.cameraCallback)
        else:
            rospy.Subscriber(camera_topic, Image, self.cameraCallback)

        cmd_topic = input("Enter cmd_vel topic: (default: /cmd_vel)")
        if cmd_topic == "":
            self.cmd_topic = "/cmd_vel"
            self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        else:
            self.cmd_topic = cmd_topic
            self.pub = rospy.Publisher(
                cmd_topic, Twist, queue_size=10)

        self.createTackerbar()

    def createTackerbar(self):
        cv2.namedWindow("HSV trackbar")
        cv2.createTrackbar("low_h", "HSV trackbar", 0, 179, self.nothing)
        cv2.createTrackbar("low_s", "HSV trackbar", 0, 255, self.nothing)
        cv2.createTrackbar("low_v", "HSV trackbar", 0, 255, self.nothing)
        cv2.createTrackbar("high_h", "HSV trackbar", 0, 179, self.nothing)
        cv2.createTrackbar("high_s", "HSV trackbar", 0, 255, self.nothing)
        cv2.createTrackbar("high_v", "HSV trackbar", 0, 255, self.nothing)

    def nothing(self, x):
        pass

    def getTrackbarPos(self):
        low_h = cv2.getTrackbarPos("low_h", "HSV trackbar")
        low_s = cv2.getTrackbarPos("low_s", "HSV trackbar")
        low_v = cv2.getTrackbarPos("low_v", "HSV trackbar")
        high_h = cv2.getTrackbarPos("high_h", "HSV trackbar")
        high_s = cv2.getTrackbarPos("high_s", "HSV trackbar")
        high_v = cv2.getTrackbarPos("high_v", "HSV trackbar")
        return low_h, low_s, low_v, high_h, high_s, high_v

    def cameraCallback(self, mesaj):
        # get trackbar positions
        # low_h, low_s, low_v, high_h, high_s, high_v = self.getTrackbarPos()
        low_h = 13
        low_s = 44
        low_v = 50
        high_h = 179
        high_s = 255
        high_v = 255

        # convert the image to opencv format
        img = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")

        img_resized = cv2.resize(
            img, (0, 0), fx=0.5, fy=0.5)  # resize the image
        # crop the image

        crop_witdth = int(img_resized.shape[1] * 0.25)
        crop_height = int(img_resized.shape[0] * 0.40)
        corp_img = img_resized[crop_height-50:,
                               crop_witdth:img_resized.shape[1] - crop_witdth]

        self.line_track(low_h, low_s, low_v, high_h,
                        high_s, high_v, img_resized)

    def line_track(self, low_h, low_s, low_v, high_h, high_s, high_v, img_resized):

        # convert the image to hsv format(hue,saturation,value)
        hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)

        # lower hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        lower = np.array([low_h, low_s, low_v])
        # upper hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        upper = np.array([high_h, high_s, high_v])

        mask = cv2.inRange(hsv, lower, upper)  # create a mask for the color
        # erode the mask to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        # dilate the mask to fill in gaps
        mask = cv2.dilate(mask, None, iterations=2)
        # blur the mask to make the edges clearer
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # bitwise and the image and the mask
        result = cv2.bitwise_and(img_resized, img_resized, mask=mask)
        h, w, d = img_resized.shape  # get the height,width and depth of the image
        rospy.loginfo("h: %d, w: %d, d: %d", h, w, d)
        # draw a circle at the center of the image to show the center of the image
        # (x,y,radius,color,thickness) -1 means filled ,color is BGR
        cv2.circle(img_resized, (int(w/2), int(h/2-20)), 5, (0, 0, 255), -1)
        M = cv2.moments(mask)  # find the moments of the mask
        if M['m00'] > 10:  # if m00 is not zero,m00 means the area of the image
            # cx is the x coordinate of the center of mass
            cx = int(M['m10']/M['m00'])
            # cy is the y coordinate of the center of mass
            cy = int(M['m01']/M['m00'])
            # draw a circle at the center of the contour to show the center of the contour
            cv2.circle(img_resized, (cx, cy), 5, (255, 0, 0), -1)
            # draw a line from the center of the contour to the center of the image
            cv2.line(img_resized, (cx-20, cy), (cx+20, cy), (255, 0, 0), 2)
            # draw a line from the center of the contour to the center of the image
            cv2.line(img_resized, (cx, cy-20), (cx, cy+20), (20, 100, 200), 2)
            # draw a line  for error correction
            cv2.line(img_resized, (cx-10, cy-10),
                     (cx-10, cy+10), (0, 255, 0), 2)  # (b, g, r)
            cv2.line(img_resized, (cx+10, cy-10),
                     (cx+10, cy+10), (0, 255, 0), 2)

            # calculate the distance from the center of the image to the center of the contour
            error = cx - w/2
            cv2.putText(img_resized, "Distance: {:.2f}cm".format(
                error), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            print(error)  # print the distance
            self.cmd_vel.linear.x = 0.4
            # set the angular velocity to the distance divided by 100
            self.cmd_vel.angular.z = -error/180
            self.pub.publish(self.cmd_vel)
        else:
            pass
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.pub.publish(self.cmd_vel)
        # put text on the image

        cv2.imshow("Orjinal", img_resized)
        cv2.imshow("Maske", mask)
        cv2.imshow("Sonuc", result)
        cv2.waitKey(1)  # wait for 1 ms

    def line_track2(self, low_h, low_s, low_v, high_h, high_s, high_v, img_resized):
        # convert the image to hsv format(hue,saturation,value)
        hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)

        # lower hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        lower = np.array([low_h, low_s, low_v])
        # upper hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        upper = np.array([high_h, high_s, high_v])

        mask = cv2.inRange(hsv, lower, upper)
        # erode the mask to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        # dilate the mask to fill in gaps
        mask = cv2.dilate(mask, None, iterations=2)
        # blur the mask to make the edges clearer
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # find the contours in the mask
        cnts = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # find the contours in the mask
        center = None  # initialize the center of the object
        if len(cnts) > 0:  # if there are contours
            c = max(cnts, key=cv2.contourArea)  # find the largest contour
            # find the center and radius of the largest contour
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # find the moments of the largest contour,moments mean the center of mass of the contour
            M = cv2.moments(c)
            # find the center of mass of the contour
            center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
            if radius > 10:  # if the radius is greater than 10
                # draw a circle around the contour
                cv2.circle(img_resized, (int(x), int(y)),
                           int(radius), (0, 255, 255), 2)
                # draw a circle at the center of the contour
                cv2.circle(img_resized, center, 5, (0, 0, 255), -1)
                self.cmd_vel.linear.x = 0.2  # set the linear velocity to 0.2
                self.cmd_vel.angular.z = 0.2  # set the angular velocity to 0.2
                self.pub.publish(self.cmd_vel)  # publish the command
        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.pub.publish(self.cmd_vel)
        cv2.imshow("mask", mask)  # show the mask
        cv2.imshow("image", img_resized)  # show the image
        cv2.waitKey(1)  # wait for 1 ms


if __name__ == "__main__":
    LineTrack()
    rospy.spin()
