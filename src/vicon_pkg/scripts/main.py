import netft_reader
import vicon_reader
import myo_reader
import rospy
from geometry_msgs.msg import WrenchStamped
from vicon_bridge.msg import Markers
from ros_myo.msg import EmgArray


#vicon采样频率上限为330左右，此处最好设置为250，并且力传感器也应该修改采样频率

if __name__ == '__main__':

    rospy.init_node("myListener",anonymous=True)
    
    rospy.Subscriber('vicon/markers', Markers, vicon_reader.vicon_callback)
    rospy.Subscriber('netft_data', WrenchStamped, netft_reader.sensor_callback)
    rospy.Subscriber('/myo_raw/myo_emg', EmgArray, myo_reader.myoReader)

    rospy.spin()