#!/usr/bin/env python
# -*- coding:utf-8 -*-
import netft_reader
import vicon_reader
import myo_reader
import rospy
from geometry_msgs.msg import WrenchStamped
from vicon_bridge.msg import Markers
from ros_myo.msg import EmgArray



if __name__ == '__main__':

    rospy.init_node("myListener",anonymous=True)
    
    
    rospy.Subscriber('netft_data', WrenchStamped, netft_reader.sensor_callback) #采样频率为250
    rospy.Subscriber('/myo_raw/myo_emg', EmgArray, myo_reader.myoReader) #采样频率为50
    rospy.Subscriber('vicon/markers', Markers, vicon_reader.vicon_callback) #采样频率为250
    rospy.spin()