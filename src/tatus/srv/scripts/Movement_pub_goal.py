#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

import rospy

from tatus.msg import Distance


def target_pub(goal):
    rospy.init_node("mesafe_publish")
    pub_dist = rospy.Publisher("mesafe_git", Distance, queue_size=10)
    target = Distance()
    target.dist = goal
    rate = rospy.Rate(1)  # 1 Hz
    # publish one the target distance
    rospy.loginfo("GİRİLEN Hedef konum : {}".format(goal))
    pub_dist.publish(target)
    rate.sleep()


try:
    goal = float(input("Hedef Konumu Giriniz : "))
    target_pub(goal)
except ValueError:
    print("Hedef Konumu Sayısal Değer Olmalıdır !")
except rospy.ROSInterruptException:
    print("Dugum sonlandi!!!")
