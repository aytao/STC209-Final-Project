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
from geometry_msgs.msg import TransformStamped
from ViconObjectVisualization import MarkersVisualization
import numpy as np

class WandTracker():
    def __init__(self):
        self.pose = np.array([[0, 0, 0]])
        self.wand_tracker_sub = rospy.Subscriber("vicon/Wand/Wand", TransformStamped, self.callback_vicon)
        rospy.init_node("wand_tracker", anonymous=True)
        self.rate = rospy.Rate(100)

    def callback_vicon(self, data):
        self.pose = np.array([[data.transform.translation.x, data.transform.translation.y, data.transform.translation.z]])
        #print(self.pose)

if __name__ == '__main__':
    # domain parameters
    xmax = 2;   xmin = -2
    zmax = 2.5; zmin = 0
    domain = np.transpose(np.array([[xmin, xmax, xmax, xmin, xmin],
                                    [zmin, zmin, zmax, zmax, zmin]]))
    # initialize visualization
    vicon_objects = np.transpose(np.array([[0], [1]]))
    visual = MarkersVisualization(domain, vicon_objects)

    # track the wand
    wand = WandTracker()

    while not rospy.is_shutdown():
        visual.update_objects(wand.pose)
    rospy.spin()
