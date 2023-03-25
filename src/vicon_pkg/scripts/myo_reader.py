#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os
import time
import rospy
import csv
from ros_myo.msg import EmgArray
import numpy as np

data_dir = os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'data')


def myoReader(myo_data):
    semg_data = []
    # data_path = os.path.join(data_dir, "myo_data.csv")
    data_path = os.path.join('/home/lgx/Project/upperlimb_robot/src/data', "myo_data2.csv")
    
    for idx, d in enumerate(myo_data.data):
        semg_data.append(d)
    semg_data = np.array(semg_data)
    data = [time.time(), semg_data]
    with open(data_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(data)

    # print(type(semg_data))
    




if __name__ == '__main__':
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('/myo_raw/myo_emg', EmgArray, myoReader)

    rospy.spin()