#!/usr/bin/env python3

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import csv
from rospy_tutorials.msg import Floats
import numpy

import os.path
import time


def talker_merenja():
    #Inicialisation of publisher who reads csv file

    pub= rospy.Publisher('weather_csv', numpy_msg(Floats), queue_size=10)
    rospy.init_node('weather_merenja', anonymous=False)
    r=rospy.Rate(10)
    file_path="/home/katarina/catkin_ws/src/weather/src/data.csv"

    #Looping through csv file and remembering last_processed_row_index, so that we can start from there
    #in next iteration

    last_processed_row_index = 0
    while not rospy.is_shutdown():
        with open(file_path, 'r') as csv_file:
            csv_reader=csv.reader(csv_file, delimiter=',')

            for i, row in enumerate(csv_reader):

                #skipping already processed data

                if i<=last_processed_row_index:
                    continue

                #updating the last processed_row_index
                
                if last_processed_row_index<i:
                    last_processed_row_index=i

                #extracting needed data from csv file

                date=row[0]
                date_array=date.split("-")
                day=int(date_array[0])
                month=int(date_array[1])
                year=int(date_array[2])
                max= int(row[1])
                min=int(row[2])
                average=float(row[3])
                a=numpy.array([day, month, year, max, min, average], dtype=numpy.float32)
                #row=row[:4]

                #publishing data and sending it to loginfo for control

                if year==2016:
                    if month<8:
                        #only seding info for first 7 months of 2016, and everything from 2017
                        rospy.loginfo("Row: {}".format(a))
                        pub.publish(a)
                    else:
                        pass
                else: 
                    rospy.loginfo("Row: {}".format(a))
                    pub.publish(a)
                r.sleep()
                
    rospy.spin()

if __name__== '__main__':
    try:
        talker_merenja()
    except rospy.ROSInterruptException:
        pass
