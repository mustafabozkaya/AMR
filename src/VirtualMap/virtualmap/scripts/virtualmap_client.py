#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from virtualmap.msg import *
from virtualmap.srv import Virtualmap, VirtualmapResponse, VirtualmapRequest
from geometry_msgs.msg import *
from nav_msgs.msg import *


if __name__ == '__main__':
    try:

        # create virtuıalmap service client and service server
        rospy.init_node('virtualmap_client')
        rospy.wait_for_service('virtualmap')
        virtualmap_service = rospy.ServiceProxy('virtualmap', Virtualmap)
        request = VirtualmapRequest()
        p1_x = int(input("Enter first point x1: "))
        p1_y = int(input("Enter first point y1: "))
        p2_x = int(input("Enter second point x2: "))
        p2_y = int(input("Enter second point y2: "))
        request.point1.x = p1_x
        request.point1.y = p1_y
        request.point1.z = 0
        request.point2.x = p2_x
        request.point2.y = p2_y
        request.point2.z = 0

        response = virtualmap_service(request)
        print(response.success)
        rospy.loginfo(response.success)
    except rospy.ROSInterruptException as e:
        print(e)
