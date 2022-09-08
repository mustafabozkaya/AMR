#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tatus.msg import Distance
from tatus.srv import CircleMove, CircleMoveRequest, CircleMoveResponse
import time
import datetime


class Movement():

    def __init__(self):
        pass

    def __init__(self, publisher_topic, subsriber_topics, speed, rate, quee_s) -> None:
        super().__init__()

        self.pub_topic = publisher_topic
        self.sub_topic = subsriber_topics
        self.speed = speed
        print("Published topic : {}".format(self.pub_topic))

        self.subodom = rospy.Subscriber(
            self.sub_topic["odom"], Odometry, self.callbackodom)
        self.subdist = rospy.Subscriber(
            self.sub_topic["dist"], Distance, self.callbackdist)
        self.pub = rospy.Publisher(self.pub_topic, Twist, queue_size=quee_s)

        self.cmd_msg = Twist()
        self.target = 0.0
        self.recent_location = 0.0
        self.start_point = 0.0
        self.rate = rospy.Rate(rate)

    def parrolling(self):

        patroling_distance = float(rospy.get_param("/parroling_distance"))
        patroling_count = int(rospy.get_param("/parroling_count"))
        count = 0
        self.start_point = self.recent_location
        rospy.loginfo("Parroling started")
        diff = 0.0
        while count < patroling_count:
            rospy.loginfo("Parroling {}".format(count))
            tstart = datetime.datetime.now()

            if count % 2 == 0:
                self.cmd_msg.linear.x = -self.speed
            else:
                self.cmd_msg.linear.x = self.speed

            while diff <= patroling_distance:
                timeend = datetime.datetime.now()
                rospy.loginfo((timeend-tstart).total_seconds())
                self.pub.publish(self.cmd_msg)
                rospy.loginfo("diff : {} ,recent_location : {} , start_point : {}".format(
                    diff, self.recent_location, self.start_point))
                diff = abs(self.recent_location - self.start_point)
                self.rate.sleep()
            self.start_point = self.recent_location
            diff = 0.0
            self.cmd_msg.linear.x = 0.0
            self.pub.publish(self.cmd_msg)
            count += 1
        rospy.loginfo("Parroling finished")

    def callbackdist(self, msgg):
        self.target = msgg.dist

    def callbackodom(self, msg):
        self.recent_location = msg.pose.pose.position.x

    def movement_cmd(self):

        # start_time = time.time()
        # distance = 10  # 10 meters
        while not rospy.is_shutdown():

            rospy.loginfo(" recent location : {} , goal : {} ...".format(
                self.recent_location, self.target))

            if (self.recent_location < self.target):

                self.cmd_msg.linear.x = self.speed  # 0.5 m/s

                self.pub.publish(self.cmd_msg)

            # elif (self.recent_location > self.target):
            #     self.cmd_msg.linear.x = -self.speed
            #     self.pub.publish(self.cmd_msg)

            else:
                self.cmd_msg.linear.x = 0.0
                self.pub.publish(self.cmd_msg)
                rospy.loginfo(" goal reached")

                # endtime = time.time()
                # distance -= cmd.linear.x*(endtime-start_time)
            self.rate.sleep()

    def client_circle_move(self, radius, speed):
        rospy.wait_for_service("circle_move_server")
        try:
            client = rospy.ServiceProxy(
                "circle_move_server", CircleMove)  # create service client object req
            req = CircleMoveRequest()  # create request object
            req.radius = radius
            req.speed = speed
            resp = client(req)  # call service client object

            rospy.loginfo("resp : {}".format(resp))
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))


def main():
    rospy.init_node("movement_publisher")
    move = Movement(
        "cmd_vel", {"odom": "odom", "dist": "distance"}, 0.5, 20, 10)
    # move.movement_cmd()
    # radius = float(input("Enter radius : "))
    # speed = float(input("Enter speed : "))
    move.parrolling()
    # move.client_circle_move(radius, speedk)  # radius, speed
    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped


if __name__ == "__main__":

    try:
        main()  # to run the code in the main function
    except rospy.ROSInterruptException:
        print('Something went wrong')
    finally:
        print('The try except is finished')
