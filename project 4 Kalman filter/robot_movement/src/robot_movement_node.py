#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_movement.srv import service, serviceResponse
from std_msgs.msg import Float64MultiArray

def response_callback(req):

	msg = Twist()
	
	# rucni rezim
	if(req.rezim=='R'):
		speed = 0.1
		angle = np.pi/18
		rospy.loginfo('Rucni rezim')
		msg.linear.x = 0
		msg.linear.y = 0
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = 0
		if(req.lin_kretanje=='w'):
			msg.linear.x = speed
		elif(req.lin_kretanje=='x'):
			msg.linear.x = -speed
		elif(req.lin_kretanje=='s'):
			msg.linear.x = 0
			msg.angular.x = 0
		if(req.rot_kretanje=='d'):
			msg.angular.z = angle
		elif(req.rot_kretanje=='a'):
			msg.angular.z = -angle	
		vel_pub.publish(msg)
			
	# automatski rezim
	if(req.rezim=='A'):
		
		global x, y, theta
		
		print('Poz x: %s',x)
		print('Poz y: %s',y)
		print('Poz theta: %s',theta)
		
		delta_x = req.x_goal-x
		delta_y = req.y_goal-y
		print('delta x: %s',delta_x)
		print('delta y: %s',delta_y)
		rho = np.sqrt((delta_x)**2+(delta_y)**2)
		print('ro: %s',rho)
		
		while (rho>= 0.001)or(abs(req.theta_goal-theta)>=0.1):
					
			alpha = - theta + math.atan2(delta_y,delta_x)
			print('alfa: %s',alpha)
			beta = - theta - alpha 
			print('beta: %s',beta)
			k_rho = 0.5 # k_rho>0
			k_alpha = 1 # k_alpha>k_rho
			k_beta = -0.5 # k_beta<0
			
			if (alpha<np.pi/2)and(alpha>-np.pi/2):
				# cilj je ispred
				v = k_rho*rho
				w = k_alpha*alpha + k_beta*beta
			else:
				# cilj je iza
				v = -k_rho*rho
				# proveriti uglove alpha i beta
				w = k_alpha*alpha+k_beta*beta
			
			
			delta_x = req.x_goal-x
			delta_y = req.y_goal-x
			rho = np.sqrt((delta_x)**2+(delta_y)**2)
			r.sleep()
	
			print('theta: %s',theta)
			msg.linear.x = np.cos(theta)*v
			msg.linear.y = np.sin(theta)*v
			msg.angular.z = w
			vel_pub.publish(msg)		
			
	return serviceResponse(True)
	
def callback(data):
	global x, y, theta
	x = data.data[0]
	y = data.data[1]
	theta = data.data[2]

	
rospy.init_node('upravljanje_robotom_kalman',anonymous=False)
vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
odom_sub = rospy.Subscriber('kalman',Float64MultiArray,callback)
s = rospy.Service('servis',service,response_callback)
r = rospy.Rate(10)
rospy.loginfo('spremno')
rospy.spin()

