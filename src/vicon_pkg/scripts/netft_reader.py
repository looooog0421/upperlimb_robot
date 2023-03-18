import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
import os
import csv
from vicon_bridge.msg import Markers
import time

data_dir = os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'data')

def forceReader(force_msg):
    force = np.zeros((1,3))
    force[0,0] = force_msg.wrench.force.x
    force[0,1] = force_msg.wrench.force.y
    force[0,2] = force_msg.wrench.force.z

    return force


def sensor_callback(WrenchStamped_msg):
    
    data_path = os.path.join(data_dir, "force_data.csv")

    t = time.time()
    force = forceReader(WrenchStamped_msg)
    data = [t,force]

    with open(data_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(data)
    
    return force


def netftReader():
    rospy.Subscriber('netft_data', WrenchStamped, sensor_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        netftReader()

        
    except rospy.ROSInterruptException:
        pass