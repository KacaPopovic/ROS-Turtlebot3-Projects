#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def callback_prikaza(data):
    
    #printing the info

    rospy.loginfo('Average daily temp is %s', data.data)

def listener_prikaza():
    rospy.init_node('weather_prikaz', anonymous= False)
    rospy.Subscriber('weather_average', Float64, callback_prikaza)
    rospy.spin()

if __name__=='__main__':
    try:
        listener_prikaza()
    except rospy.ROSInterruptException:
        pass
