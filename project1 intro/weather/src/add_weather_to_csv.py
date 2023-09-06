#!/usr/bin/env python3

import rospy
from weather.srv import add_weather_to_csv, add_weather_to_csvResponse
import csv
import datetime

def response_callback(req):
    file_path="/home/katarina/catkin_ws/src/weather/src/data.csv"
    try:

        #initialising last_row in order to avoid exceptions

        last_row=[]
        csv_file=open(file_path, 'r')
        csv_reader=csv.reader(csv_file,delimiter=',')

        #finding the last row in csv file

        for row in csv_reader:
            last_row=row
    except Exception as e:
        print("Error: ",e)

    #finding the next date after the last date in csv file

    date=last_row[0]
    csv_file.close()
    date_array=date.split("-")
    day=int(date_array[0])
    month=int(date_array[1])
    year=int(date_array[2])
    last_date=datetime.date(year,month, day)
    next_date=last_date+datetime.timedelta(days=1)
    next_date_str="{:02d}-{:02d}-{:04d}".format(next_date.day, next_date.month, next_date.year)

    #adding new row in csv file with next_date_str, temps converted to celsius and zeros for unneeded elements
    
    csv_file=open(file_path, 'a+')
    csv_writer=csv.writer(csv_file, delimiter=',')
    min=(req.min*1.8)+32
    max=(req.maks*1.8)+32
    min=int(min)
    max=int(max)
    aver=(req.average*1.8)+32
    csv_writer.writerow([next_date_str, str(max),str(min),str(aver), "0","0", "0"])
    csv_file.close()
    return add_weather_to_csvResponse(True)

rospy.init_node('add_weather_to_csv_node')
s=rospy.Service('add_weather_to_csv', add_weather_to_csv, response_callback)
rospy.loginfo('Service is ready')
rospy.spin()