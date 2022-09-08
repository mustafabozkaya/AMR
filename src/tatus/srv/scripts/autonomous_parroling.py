#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Uygulama 6: Lazer Sensörden Gelen Veriyi Kullanma
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class LaserData():
    def __init__(self):
        rospy.init_node("lazer_dugumu")
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.hiz_mesaji = Twist()
        self.front = 0.0
        self.left = 0.0
        self.right = 0.0
        self.back = 0.0
        self.base_radius = 0.2
        rospy.Subscriber("scan", LaserScan, self.lazerCallback)
        rospy.spin()

    def lazerCallback(self, mesaj):
        thresh_bf = 1.2
        thresh_near = 1.0
        sol_on = np.mean(list(mesaj.ranges[0:9]))
        sag_on = np.mean(list(mesaj.ranges[350:359]))
        on = sol_on + sag_on
        sol = np.mean(list(mesaj.ranges[80:100]))
        sag = np.mean(list(mesaj.ranges[260:280]))
        arka = np.mean(list(mesaj.ranges[170:190]))
        print("sol:", sol, "sol ön ", sol_on, "sag_ön",
              sag_on, "sag:", sag, "arka:", arka)

        if sol_on < thresh_bf and sag_on < thresh_bf:

            self.hiz_mesaji.linear.x = -0.1
            self.hiz_mesaji.angular.z = 1/self.base_radius
            self.pub.publish(self.hiz_mesaji)
        elif sol_on < thresh_bf and sag_on > thresh_bf:

            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = -1/self.base_radius
            self.pub.publish(self.hiz_mesaji)

        elif on > thresh_near and sol > thresh_near or sag > thresh_near:
            self.hiz_mesaji.linear.x = 0.5
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)

        elif on > thresh_bf and sol < thresh_near and sag < thresh_near:
            self.hiz_mesaji.linear.x = 0.5
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)
        else:
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)


nesne = LazerVerisi()
