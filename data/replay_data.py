import os
import csv
import numpy as np

import rospy

import tf
import tf2_ros
import geometry_msgs.msg

from ds4_driver.msg import Status

class ReplayData:
    def __init__(self, name):
        self.arm_status = []
        self.base_status = []
        self.handover_status = []

        with open(name + ".csv", newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.arm_status += [row["arm status"]]
                self.base_status += [row["base status"]]
                self.handover_status += [row["handover status"]]

        self.i = 0
        self.N = len(self.arm_status)
        self.publish_rate = 100
        self.csv_rate = 10

        self.status_pub = rospy.Publisher("/status", Status, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1 / self.publish_rate), self.timer_callback)


    def timer_callback(self, event):
        status = Status()

        i = int(np.floor(self.i))

        if self.arm_status[i] == "REACHING":
            status.axis_right_y = 1.0
            status.button_l1 = 1
            status.button_r1 = 1
        elif self.arm_status[i] == "TUCKING":
            status.axis_right_y = -1.0
            status.button_l1 = 1
            status.button_r1 = 1

        if self.base_status[i] == "TO PARTICIPANT" or self.base_status[i] == "TO OPERATOR":
            status.axis_right_y = 1.0
            status.button_l1 = 1
        elif self.base_status[i] == "ROTATING":
            status.axis_left_x = 1.0
            status.button_l1 = 1

        if self.handover_status[i] == "LEFT":
            status.button_square = 1
            status.button_l1 = 1
        elif self.handover_status[i] == "MIDDLE":
            status.button_triangle = 1
            status.button_l1 = 1
        elif self.handover_status[i] == "RIGHT":
            status.button_circle = 1
            status.button_l1 = 1

        self.status_pub.publish(status)

        self.i = (self.i + self.csv_rate / self.publish_rate) % self.N



if __name__ == "__main__":
    name = input("Enter prefix of bag to convert to CSV: ")

    rospy.init_node("replay_node")
    
    replay_data = ReplayData(name)

    rospy.spin()