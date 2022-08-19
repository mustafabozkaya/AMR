#!/usr/bin/env python3
# license removed for brevity
from ast import Str
from random import random
import rospy
from std_msgs.msg import String,Float32
from geometry_msgs.msg import Twist
#from tatu.msg import Movement_cmd


class Movement(object):

    def __init__(self) -> None:
        rospy.init_node('line_tracker_n', anonymous=True)
        self.pub = rospy.Publisher("line_track", String, 10)
        self.rate = rospy.Rate(10)  # 10hz

    def callback_line(self, msg):
        command = msg.data

        if str.lower(command) == "left":

            print("left")
        elif str.lower(command) == "right":
            print("right")
        else:
            print("on straight")

    def talker(self):
        while not rospy.is_shutdown():

            line_msg = String()
            line_msg = "linear hiz: 100, angular: 2"

            vel=Float32()
            vel.data=1.0 

            arr=[]
            arr.append(vel.data*1) #1 .motor velocity
            arr.append(vel.data/2)  # 2 .motor velocity
            arr.append(vel.data/3)  # 3 .motor velocity
            arr.append(vel.data*1)  # 4 .motor velocity


            #rospy.loginfo(line_msg)

            self.pub.publish(arr)
            self.rate.sleep()

    def listener(self):
        sub = rospy.Subscriber("/movement_cmd", String, self.callback_line)
        rospy.spin()

    def main(self):
        try:
            self.talker()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    line = Movement()
    line.main()
