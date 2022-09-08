#!/usr/bin/env python3

import math
import rospy
import cv2
import random
from std_msgs.msg import Float32


cam = cv2.VideoCapture("./line_video.mp4")
ret, frame = cam.read()

frame_height = frame.shape[0]
frame_width = frame.shape[1]
start_offset = 100
end_offset = 100

rectangle_start_point = (int(start_offset), int(start_offset))
rectangle_end_point = (int(frame_width - end_offset),
                       int(frame_height - end_offset))
rectangle_color = (0, 255, 0)
rectangle_thickness = 1

target_coordinates = (random.randint(0, frame_width),
                      random.randint(0, frame_height))
target_radius = 5
target_color = (0, 0, 255)
target_thickness = target_radius * 2

origin_coordinates = (int(frame_width / 2), int(frame_height / 2))
origin_radius = 10
origin_color = (255, 0, 0)
origin_thickness = 2


while True:
    ret, frame = cam.read()
    rand_x = random.randint(0, frame_width)
    rand_y = random.randint(0, frame_height)
    target_coordinates = (rand_x, rand_y)
    target_dist_x = rand_x - int(frame_width / 2)
    target_dist_y = rand_y - int(frame_height / 2)

    if target_dist_x != 0:
        aci_degeri = math.degrees(math.atan2(target_dist_x, target_dist_y))
        print(aci_degeri)
    else:
        aci_degeri = 0

    edited_frame = cv2.rectangle(
        frame,
        rectangle_start_point,
        rectangle_end_point,
        rectangle_color,
        rectangle_thickness,
    )
    edited_frame = cv2.circle(
        edited_frame, target_coordinates, target_radius, target_color, target_thickness
    )
    edited_frame = cv2.circle(
        edited_frame, origin_coordinates, origin_radius, origin_color, origin_thickness
    )
    edited_frame = cv2.line(
        edited_frame,
        origin_coordinates,
        target_coordinates,
        origin_color,
        origin_thickness,
    )

    cv2.imshow("edited frame", edited_frame)
    cv2.waitKey(5000)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cam.release()
cv2.destroyAllWindows()
