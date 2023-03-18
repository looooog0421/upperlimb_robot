import rospy
import sys
import numpy as np
from vicon_bridge.msg import Markers

import os
import csv
import time

data_dir = os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'data')

def pointReader(point_raw):
    points = np.zeros((7,3))
    for i in range(12):
        if point_raw.markers[i].marker_name == 'A':
            points[0,0] = point_raw.markers[i].translation.x
            points[0,1] = point_raw.markers[i].translation.y
            points[0,2] = point_raw.markers[i].translation.z
        if point_raw.markers[i].marker_name == 'B':
            points[1,0] = point_raw.markers[i].translation.x
            points[1,1] = point_raw.markers[i].translation.y
            points[1,2] = point_raw.markers[i].translation.z
        if point_raw.markers[i].marker_name == 'C':
            points[2,0] = point_raw.markers[i].translation.x
            points[2,1] = point_raw.markers[i].translation.y
            points[2,2] = point_raw.markers[i].translation.z
        if point_raw.markers[i].marker_name == 'D1':
            points[3,0] = point_raw.markers[i].translation.x
            points[3,1] = point_raw.markers[i].translation.y
            points[3,2] = point_raw.markers[i].translation.z
        if point_raw.markers[i].marker_name == 'D2':
            points[4,0] = point_raw.markers[i].translation.x
            points[4,1] = point_raw.markers[i].translation.y
            points[4,2] = point_raw.markers[i].translation.z
        if point_raw.markers[i].marker_name == 'E':
            points[5,0] = point_raw.markers[i].translation.x
            points[5,1] = point_raw.markers[i].translation.y
            points[5,2] = point_raw.markers[i].translation.z
        if point_raw.markers[i].marker_name == 'F':
            points[6,0] = point_raw.markers[i].translation.x
            points[6,1] = point_raw.markers[i].translation.y
            points[6,2] = point_raw.markers[i].translation.z

    points = points.flatten()
    points = points.reshape(1,-1)

    return points
    


def vicon_callback(Point_msg):

    data_path = os.path.join(data_dir, "vicon_data.csv")

    points = pointReader(Point_msg)
    t = time.time()
    data = [t, points]

    with open(os.path.join(data_path), 'a') as f:
        writer = csv.writer(f)
        writer.writerow(data)
    

    return points


def viconReader():
    
    rospy.Subscriber('vicon/markers', Markers, vicon_callback)
    rospy.spin()



if __name__ == "__main__":
    try:
        viconReader()

        
    except rospy.ROSInterruptException:
        pass


