#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64, Int64
import numpy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

counter=0

def callback_akcije(data):

    #printing info when it changes
    
    global counter
    plus= int(data.data)
    counter=counter+plus
    if plus==1:
        rospy.loginfo('Number of days with high difference %s', counter)

def listener_ackije():
    rospy.init_node('weather_akcija', anonymous=False)
    rospy.Subscriber('weather_diff', String, callback_akcije)
    rospy.spin()

if __name__=='__main__':
    try:
        listener_ackije() 
    except rospy.ROSInterruptException:
        pass 