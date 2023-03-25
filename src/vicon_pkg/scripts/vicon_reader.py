#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import sys
import numpy as np
from vicon_bridge.msg import Markers

import os
import csv
import time

data_dir = os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'data')

def pointReader(point_raw):
    # points = np.zeros((7,3))
    points = [time.time()]
    for i in range(12):
        if point_raw.markers[i].marker_name == 'A':
            points.append(point_raw.markers[i].translation.x)
            points.append(point_raw.markers[i].translation.y)
            points.append(point_raw.markers[i].translation.z)
        if point_raw.markers[i].marker_name == 'B':
            points.append(point_raw.markers[i].translation.x)
            points.append(point_raw.markers[i].translation.y)
            points.append(point_raw.markers[i].translation.z)
        if point_raw.markers[i].marker_name == 'C':
            points.append(point_raw.markers[i].translation.x)
            points.append(point_raw.markers[i].translation.y)
            points.append(point_raw.markers[i].translation.z)
        if point_raw.markers[i].marker_name == 'D1':
            points.append(point_raw.markers[i].translation.x)
            points.append(point_raw.markers[i].translation.y)
            points.append(point_raw.markers[i].translation.z)
        if point_raw.markers[i].marker_name == 'D2':
            points.append(point_raw.markers[i].translation.x)
            points.append(point_raw.markers[i].translation.y)
            points.append(point_raw.markers[i].translation.z)
        if point_raw.markers[i].marker_name == 'E':
            points.append(point_raw.markers[i].translation.x)
            points.append(point_raw.markers[i].translation.y)
            points.append(point_raw.markers[i].translation.z)
        if point_raw.markers[i].marker_name == 'F':
            points.append(point_raw.markers[i].translation.x)
            points.append(point_raw.markers[i].translation.y)
            points.append(point_raw.markers[i].translation.z)

    # points = points.flatten()
    # points = points.reshape(1,-1)

    return points
    


def vicon_callback(Point_msg):

    data_path = os.path.join(data_dir, "vicon_data2.csv")

    points = pointReader(Point_msg)
    # t = time.time()
    # data = [t, points]

    with open(os.path.join(data_path), 'a') as f:
        writer = csv.writer(f)
        writer.writerow(points)
    

    return points


def viconReader():
    # rospy.init_node("myListener",anonymous=True)
    rospy.Subscriber('vicon/markers', Markers, vicon_callback)
    rospy.spin()



if __name__ == "__main__":
    try:
        rospy.init_node("myListener",anonymous=True)
        viconReader()

        
    except rospy.ROSInterruptException:
        pass


