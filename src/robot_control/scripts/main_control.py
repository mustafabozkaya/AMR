#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Main_Control:
    def __init__(self):
        self.encoder_sub = rospy.Subscriber('encoder', Int32, self.encoder_callback)
        self.lift_pub = rospy.Publisher('lift', Bool, queue_size=10)
        self.buzzer_pub = rospy.Publisher('buzzer', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd_vel_msg = Twist()
        self.encoder_count = 0
        self.prev_encoder_count = 0
        self.speed = 0.25
        self.radius = 0.5
        self.publish_cmd = False
        self.rate = rospy.Rate(10)
        self.t0 = rospy.Time.now().to_sec()
        self.t1 = rospy.Time.now().to_sec()
        self.yer_degistirme = 0
        self.hiz_mesaji = Twist()
        self.robot_hiz = 0.25
        self.volta_uzunluk = rospy.get_param("/parroling_distance")
        self.volta_sayisi = rospy.get_param("/parroling_count")
        self.sayici = 0
        self.rospy.loginfo("Devriye gezmeye baslandi !")
        self.rospy.is_shutdown()
        self.rospy.spin()
        self.rospy.loginfo("Devriye tamamlandi !")
        self.rospy.is_shutdown()
        self.rospy.spin()
        self.rospy.loginfo("Devriye tamamlandi !")
        self.rospy.is_shutdown()
        self.rospy.spin()
        self.rospy.loginfo("Devriye tamamlandi !")
        self.rospy.is_shutdown()
        self.rospy.spin()
        self.rospy.loginfo("Devriye tamamlandi !")
        self.rospy.is_shutdown()
        self.rosp