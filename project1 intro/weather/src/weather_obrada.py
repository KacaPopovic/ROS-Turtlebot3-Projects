#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64, Int64
import numpy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import csv

def callback_obrade(data):
    #rospy.loginfo('Average daily temp is %s', data.data[4])
    talker_obrade(data.data)

def talker_obrade(data):
    #initialising 2 publishers, one for average temperature and another for min max difference

    pub_average=rospy.Publisher('weather_average', Float64, queue_size=10)
    pub_diff=rospy.Publisher('weather_diff', String, queue_size=10)
    r=rospy.Rate(10)

    #data processing far to cels

    day=data[0]
    month=data[1]
    year=data[2]
    max=(data[3]-32)/1.8
    min=(data[4]-32)/1.8
    average=(data[5]-32)/1.8
    a=numpy.array([max, min, average], dtype=numpy.float32)

    #publishing

    pub_average.publish(average)
    if (abs(max-min)>15):
        pub_diff.publish("1")
    else:
        pub_diff.publish("0")

    #writing processed data in new csv
    
    with open('processed_data.csv', mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([day, month, year, max, min, average])

def listener_obrade():
    rospy.init_node('weather_obrada', anonymous=False)
    rospy.Subscriber('weather_csv', numpy_msg(Floats), callback_obrade)
    rospy.spin()

if __name__=='__main__':
    try:
        listener_obrade() 
    except rospy.ROSInterruptException:
        pass 