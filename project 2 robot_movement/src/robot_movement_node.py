#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_movement.srv import robot_movement_service, robot_movement_serviceResponse


def robot_movement_go(data):

    #Create msg
    msg = Twist()

    # Manual mode
    if(data.mode == 'M'):
        rospy.loginfo('MAN mode')
        linear_speed = 0.3
        angular_speed = np.pi/20
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        
        if(data.linear_direction == 'forward'): # forward
            msg.linear.x = linear_speed
        elif(data.linear_direction == 'backward'): # backward
            msg.linear.x = -linear_speed
        elif(data.linear_direction == 'stop'): # stand-by
            msg.linear.x = 0
        if(data.angular_direction == 'left'): # left rotation
            msg.angular.z = -angular_speed
        elif(data.angular_direction == 'right'): # right rotation
            msg.angular.z = angular_speed	
        pub.publish(msg)


    # Automatic mode
    if(data.mode == 'A'):
        rospy.loginfo('AUTO mode')
        global x, y, theta
        x_t = data.target_x # target x-axis value
        y_t = data.target_y # target y-axis value
        theta_t = data.target_theta # target orientation
        delta_x = x_t-x
        delta_y = y_t-y
        rho_t = np.sqrt((delta_x)**2 + (delta_y)**2) # target distance

        while ((rho_t>0.05) or (abs(theta_t-theta)>0.1)):
            
            # Calculating angles and setting parameters
            alpha = -theta + math.atan2(delta_y,delta_x) # fist rotation angel
            beta = -(theta + alpha) + theta_t # second rotation angel      
            k_rho = 0.3  # k_rho>0
            k_alpha = 0.6  # k_alpha>k_rho
            k_beta = -0.2  # k_beta<0
            
            # Setting values for linear and angluar speed
            if((alpha>np.pi/2) or (alpha<-np.pi/2)): # target is behind robot
                v = -k_rho*rho_t 
                alpha = alpha - np.pi*np.sign(alpha) # updating alpha
                beta = beta - np.pi*np.sign(beta) # updating beta        
                w = k_alpha*alpha + k_beta*beta
            else: # target is in front of robot
                v = k_rho*rho_t
                w = k_alpha*alpha + k_beta*beta              

            #Updating new robot position
            delta_x = x_t-x
            delta_y = y_t-y
            rho_t = np.sqrt((delta_x)**2 + (delta_y)**2)

            # Maintaining constant linear speed
            if(rho_t>0.2):
                ratio = abs(v/w)
                v = 0.3*np.sign(v)
                w = 0.3/ratio*np.sign(w)
       
            #Updating msg to be publish
            msg.linear.x = v
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = w

            #Publish msg
            pub.publish(msg)

        #Updating msg to stop robot
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0    

        #Send it to stop the robot
        pub.publish(msg)

    #Return response to service caller
    return robot_movement_serviceResponse(True)



# Collect data from /odom topic
def odometryCallback(dataOdom):
    global x, y, theta
    q_x = dataOdom.pose.pose.orientation.x
    q_y = dataOdom.pose.pose.orientation.y
    q_z = dataOdom.pose.pose.orientation.z
    q_w = dataOdom.pose.pose.orientation.w
    
    x = dataOdom.pose.pose.position.x
    y = dataOdom.pose.pose.position.y
    theta = math.atan2(2.0*(q_w*q_z + q_x*q_y), 1.0 - 2.0*(q_y**2 + q_z**2))



if __name__ == '__main__':
    #Init node
    rospy.init_node("move_robot")

    #Create publisher to /cmd_vel topic
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    rospy.loginfo("Publisher is ready!")

    #Create subscriber to /odom
    sub = rospy.Subscriber("odom",Odometry,odometryCallback)
    rospy.loginfo("Subscriber is ready!")

    #Create service for moveing robot
    service = rospy.Service("robot_movement_service",robot_movement_service,robot_movement_go)
    rospy.loginfo("Service is ready!")

    rospy.spin()