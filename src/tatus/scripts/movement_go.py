#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
app 2:  moving through only one axis(x) at a time
"""

from dis import dis
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tatus.msg import Distance


class MovementGo():
    def __init__(self):
        rospy.init_node("duz_git")
        self.hedef_konum = 0.0
        self.guncel_konum = 0.0
        self.kontrol = True

        self.direction = "up"
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        rospy.Subscriber("mesafe_git", Distance, self.mesafeCallback)
        self.rate = rospy.Rate(10)
        self.cmd_vel_publish()

    def cmd_vel_publish(self):
        cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        hiz_mesaji = Twist()

        while not rospy.is_shutdown():
            if self.kontrol and self.direction == "up":
                hiz_mesaji.linear.x = 0.5
                cmd_pub.publish(hiz_mesaji)

            elif self.kontrol and self.direction == "down":
                hiz_mesaji.linear.x = -0.5
                cmd_pub.publish(hiz_mesaji)

            else:
                hiz_mesaji.linear.x = 0.0
                cmd_pub.publish(hiz_mesaji)
                rospy.loginfo("Hedefe varildi !")
            self.rate.sleep()

    def odomCallback(self, mesaj):
        self.guncel_konum = mesaj.pose.pose.position.x
        rospy.loginfo("absolute position : {}".format(self.guncel_konum))

        if abs(self.hedef_konum) == self.hedef_konum:

            if self.guncel_konum <= self.hedef_konum:
                self.kontrol = True
                self.direction = "up"
            else:
                self.kontrol = False
                self.direction = "up"
        else:
            if self.guncel_konum <= self.hedef_konum:
                self.kontrol = True
                self.direction = "down"
            else:
                self.kontrol = True
                self.direction = "down"

    def mesafeCallback(self, mesaj):
        self.hedef_konum = mesaj.dist
        rospy.loginfo("Hedefe {} metre uzaklikta".format(
            abs(self.hedef_konum-self.guncel_konum)))


try:
    target = MovementGo()
except rospy.ROSInterruptException:
    print("Dugum sonlandi!!!")
