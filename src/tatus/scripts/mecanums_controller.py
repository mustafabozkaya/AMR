#!/usr/bin/env python3
# license removed for brevity
from random import random
import rospy
from std_msgs.msg import String
#from tatu.msg import Movement_cmd

class Movement(object):

    def __init__(self) -> None:
        pass
def callback(msg):
    command = msg.data

    if str.lower(command) == "left":

        print("left")
    elif str.lower(command) == "right":
        print("right")
    else:
        print("on straight")




def talker():
    pub = rospy.Publisher('movement_cmd', String, queue_size=10)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        msg = String()
        rand = int(random()*100)

        if rand % 2 == 0:
            msg.data = "left"
        else:
            msg.data = "right"

        rospy.loginfo(msg)
        # pub.publish(msg)
        rate.sleep()


def listener():
    sub = rospy.Subscriber("/line_track", String, callback)
    rospy.spin()


def main():
    try:
        rospy.init_node('talker', anonymous=True)
        talker()
        listener()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
