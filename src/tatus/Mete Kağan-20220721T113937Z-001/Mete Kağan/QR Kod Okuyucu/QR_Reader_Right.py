#!/usr/bin/env python
import serial
import sys
import rospy
from std_msgs.msg import String


class QR_Point:
    name = ""
    section = ""
    X = 0
    Y = 0
    speed = 0
    distance = 0
    counter = ""

    def __init__(self, row):
        self.name = row[0]
        self.section = row[1]
        self.X = int(row[2])
        self.Y = int(row[3])
        self.speed = float(row[4])
        self.distance = row[5]
        self.counter = row[6]

    def __str__(self):
        return self.name + ": " + self.section + " (" + str(self.X) + "," + str(self.Y) + ") " + str(self.speed) + " " + str(self.distance) + " " + self.counter

    @staticmethod
    def find_QR(counter):
        for item in QR_list:
            if item.name == counter:
                return item


def handleForceQR(data):
    global QR_list

    data = data.data
    for item in QR_list:
        if data == item.name:
            print(item)
            msg = String()
            msg.data = str(item.name) + "," + str(item.speed) + \
                "," + str(item.distance) + "," + str(item.section)
            pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('QR_Reader_Right')
    pub = rospy.Publisher('QR_Command', String, queue_size=10)
    rospy.Subscriber("Force_QR", String, handleForceQR)

    port = rospy.get_param('/QR_Reader_Right/port', default="/dev/ttyUSB0")

    ser = serial.Serial(port, bytesize=8, baudrate=115200)

    QR_list = []

    with open(r"/home/eray/otonom_depo/scripts/newQRPoints.txt", "r") as file:
        lines = file.readlines()
        for line in lines:
            row = line.split(",")
            QR_list.append(QR_Point(row))

    print("Listining to port...", port)

    while not rospy.is_shutdown():
        try:
            stt = ""
            # print(ser.inWaiting())
            if ser.inWaiting() > 0:
             data = ser.read().decode("utf-8")
             while data != "\r":
                stt += data
                data = ser.read().decode("utf-8")
             ser.read()
             print(stt)
             for item in QR_list:
                if stt == item.name:
                    print(item)
                    msg = String()
                    msg.data = str(item.name) + "," + str(item.speed) + \
                        "," + str(item.distance) + "," + str(item.section)
                    pub.publish(msg)
        except Exception as e:
            print(e)