#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
app2 : move to around a circle with a specified radius
"""

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tatus.msg import Distance
from tatus.srv import CircleMove, CircleMoveRequest, CircleMoveResponse


publish_cmd= False

def circle_move(req):
    qrcode=req.qrcode
    left_encoder=req.left_encoder
    right_encoder=req.right_encoder
    rospy.loginfo("circle move service called")
    cmd_msg.linear.x = 0.4
    #w = v*tan(theta)
    # w=v/radius
    # cmd_msg.angular.z = speed/radius

    while not rospy.is_shutdown():
        if publish_cmd:

            pub.publish(cmd_msg)
            rate.sleep()

    
def pwm_to_distance(pwm):
    return pwm/18.5 # cm

def move_go(left_encoder,right_encoder):
    distance=pwm_to_distance(left_encoder)




def callback(msg):
    recent_encoderleft = msg.data.split(',')[0] # get the left encoder
    recent_encoderright = msg.data.split(',')[1] # get the right encoder

    global left_encoder
    global right_encoder
  
    
    


if __name__ == '__main__':

    rospy.init_node("circle_move_server")
    cmd_msg = Twist()
    speed = 0.5
    rate = rospy.Rate(10)
    pub_topic = "cmd_vel"
    sub_topic = "encoder_data"
    pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
    sub = rospy.Subscriber(sub_topic, String, callback)
    move_service = rospy.Service(
        "circle_move_server", CircleMove, circle_move)


    rospy.spin()


