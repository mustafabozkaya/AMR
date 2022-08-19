#!/usr/bin/env python3
# *_* coding: utf-8 *_*


import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import sys
import rospy
from geometry_msgs.msg import Point
from virtualmap.msg import Virtualpoints
from virtualmap.srv import Virtualmap, VirtualmapResponse, VirtualmapRequest


class Virtual_Draw(object):

    def __init__(self):

        # create a point variable
        self.virtualpoints = Virtualpoints()
        self.point = Point()
        self.img_path = "/virtualmap/map/map.png"
        # create a service server
        rospy.init_node('virtualmap_server')
        rospy.Service('virtualmap', Virtualmap,
                      self.handle_virtualmap_service)

        self.pub = rospy.Publisher(
            "/virtualwallpoints", Virtualpoints, queue_size=10)
        """
        Subcribe the point from geometry_msgs/Point.
        """

        rospy.Subscriber("/wallpoints", Point, self.callback)

    def handle_virtualmap_service(self, req):
        print("call_virtualmap_service")
        print("req: ", req)
        vpoints = [(req.point1), (req.point2)]
        print(vpoints[0])
        print(type(vpoints[0]))
        result = self.main(self.img_path, vpoints)
        response = VirtualmapResponse()  # create response
        response.success = result  # set success
        return response  # return response

    def draw_wall_map(self, map_img):
        """
        Draws a wall map image.
        """

        plt.imshow(map_img, cmap='gray')
        plt.show()

    def publish_vpoints(self):
        """
        Publish virtual points.
        """

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.pub.publish(self.virtualpoints)
            rate.sleep()

    def callback(self, data):
        """
        Callback function for the subscriber.
        """
        print(f"point1 {data.x, data.y}")

        global virtualpoints
        global point

        point.x = data.x
        point.y = data.y
        point.z = data.z
        print("point1: ", point)
        virtualpoints.vpoints.append(point)  # add point end of list of points
        # only last two points are needed,other are removed
        while len(virtualpoints.vpoints) > 2:
            # remove first point of list of points
            virtualpoints.vpoints.pop(0)
            if len(virtualpoints.vpoints) == 2:
                break

        print("virtualpoints: ", virtualpoints.vpoints)

        self.publish_vpoints()

    def main(self, img_path, vpoints):
        try:
            # read image
            """
            Main function.
            """
            img = cv.imread(
                img_path, cv.IMREAD_COLOR)  # read image as color image
            (h, w) = img.shape[:2]  # get image height and width
            print(f"Image size: {w}x{h}")
            # draw wall on image
            # img[h//2, w//2-60:w//2+60] = (0, 0, 0)  # draw black line on image
            # draw line on img color black linetype is dashed
            # get input line points
            print(
                f"line points (x1, y1, x2, y2) -- x range: [0, {w}], y range: [0, {h}]")
            # p1_x = int(input("Enter first point x1: "))
            # p1_y = int(input("Enter first point y1: "))
            # p2_x = int(input("Enter second point x2: "))
            # p2_y = int(input("Enter second point y2: "))
            # if virtualwallpoints topic is published, the points are received from there
            # if "virtualwallpoints" in rospy.get_published_topics():
            #     print("virtualwallpoints topic is published")

            #     (p1_x, p1_y) = vpoints[0]
            #     (p2_x, p2_y) = vpoints[1]
            # else:
            #     print("virtualwallpoints topic is not published")
            #     p1_x = int(input("Enter first point x1: "))
            #     p1_y = int(input("Enter first point y1: "))
            #     p2_x = int(input("Enter second point x2: "))
            #     p2_y = int(input("Enter second point y2: "))
            (p1_x, p1_y) = (vpoints[0].x, vpoints[0].y)
            (p2_x, p2_y) = (vpoints[1].x, vpoints[1].y)

            # check input points
            if p1_x < 0 or p1_x > w or p1_y < 0 or p1_y > h:
                print("Invalid first point")
                sys.exit(1)
            if p2_x < 0 or p2_x > w or p2_y < 0 or p2_y > h:
                print("Invalid second point")
                sys.exit(1)
            p1 = (int(p1_x), int(p1_y))
            p2 = (int(p2_x), int(p2_y))
            cv.line(img, p1, p2, (0, 0, 0), 3, cv.LINE_8)
            # write img same name as input image
            img_path = img_path.split("/")[:-1]
            # concanate all list elements of img_path
            img_path = "/".join(img_path)

            img_path = img_path + "/" + "virtualmap.png"
            print(img_path)
            cv.imwrite(img_path, img)
            # convert to multiarray
            # img_array = np.asarray(img, dtype=np.uint8)
            # cv.imshow('image', img)
            self.draw_wall_map(img)  # draw image
            cv.waitKey(0)
            cv.destroyAllWindows()
            return True
        except Exception as e:
            print(e)
            return False


if __name__ == '__main__':

    # input image path from command line
    args = sys.argv  # get all arguments
    print(args)
    if not rospy.is_shutdown():
        try:
            virtualdraw = Virtual_Draw()

            if len(args) >= 2:
                virtualdraw.img_path = args[1]
                print(f"image path: {virtualdraw.img_path}")

            else:
                print("Usage: python draw_map.py $(rospack find pkg)/maps/map.png")
                sys.exit(1)
        except rospy.ROSInterruptException:
            print("program interrupted before completion")

    rospy.spin()
