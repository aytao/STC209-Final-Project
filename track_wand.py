#!/usr/bin/env python

'''--------------------------------------------------------------------------
Title: STC 209 - Spring 2023: Track movement of hand holder in the StudioLab
Author: Maria Santos, maria.santos@princeton.edu
Description: this script subscribes to the position of a holder.
Each callback updates the visualization
Date: March 3, 2023

After having launch vicon_bridge (roslaunch vicon.launch):
>> python3 stc209.py
-------------------------------------------------------------------------'''

import rospy
import time
from geometry_msgs.msg import TransformStamped
import numpy as np

class WandTracker():
    def __init__(self):
        self.wand_pose = np.array([[0, 0, 0]])
        self.wand_tracker_sub = rospy.Subscriber("vicon/Wand/Wand", TransformStamped, self.callback_vicon_wand)
        self.other_wand_tracker_sub = rospy.Subscriber("vicon/Other_Wand/Other_Wand", TransformStamped, self.callback_vicon_other_wand)
        rospy.init_node("wand_tracker", anonymous=True)
        self.rate = rospy.Rate(100)

    def callback_vicon_wand(self, data):
        self.wand_pose = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
        #print(self.pose)
    def callback_vicon_other_wand(self, data):
        self.other_wand_pose = np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])

    def distance(self):
        return np.linalg.norm(self.other_wand_pose - self.wand_pose)

if __name__ == '__main__':
    # track the wand
    wand = WandTracker()

    while not rospy.is_shutdown():
        #time.sleep(1)
        print(wand.wand_pose, wand.other_wand_pose, wand.distance())
    rospy.spin()
