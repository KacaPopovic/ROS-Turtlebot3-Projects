#!/usr/bin/env python3

import rospy
import math
import numpy as np
import time as t
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegment,LineSegmentList
from scipy.linalg import block_diag


def transitionFunction(x_t_1,u_t,b):
	# print('Usao u transition')
	teta_t_1 = x_t_1[2];
	delta_s_l = u_t[0]; delta_s_r = u_t[1];  
	pom_mat = np.array([(delta_s_l+delta_s_r)/2*np.cos(teta_t_1+(delta_s_l-delta_s_r)/(2*b)),\
	(delta_s_l+delta_s_r)/2*np.sin(teta_t_1+(delta_s_l-delta_s_r)/(2*b)),\
	(delta_s_l-delta_s_r)/(2*b)]);
	x_t_pred = x_t_1+pom_mat;
	teta_t = x_t_pred[2]; 
	arg = teta_t+(delta_s_r-delta_s_l)/(2*b);
	F_x_pred = np.array([[1,0,-(delta_s_l+delta_s_r)/2*np.sin(arg)],[0,1,(delta_s_l+delta_s_r)/2*np.cos(arg)],[0,0,1]]);
	
	F_u_pred = np.array([[1/2*np.cos(arg)+1/(2*b)*np.sin(arg)*(delta_s_l+delta_s_r)/2,1/2*np.cos(arg)-1/(2*b)*np.sin(arg)*(delta_s_l+delta_s_r)/2],\
	[1/2*np.sin(arg)-1/(2*b)*np.cos(arg)*(delta_s_l+delta_s_r)/2,1/2*np.sin(arg)+1/(2*b)*np.cos(arg)*(delta_s_l+delta_s_r)/2],[-1/b,1/b]]);

	return x_t_pred, F_x_pred, F_u_pred
	
		
def measurmentFunction(x_t_pred,mapa_i):
	# print('Usao u measurment')
	x_t = x_t_pred[0];
	y_t = x_t_pred[1];
	teta_t = x_t_pred[2];
	ro = mapa_i[0];
	alfa = mapa_i[1]; 
	z_t_i_pred = np.array([alfa - teta_t, ro-(x_t*np.cos(alfa)+y_t*np.sin(alfa))])
	#z_t_i_pred = np.array([ro-(x_t*np.cos(alfa)+y_t*np.sin(alfa)), alfa - teta_t])
	H_x_pred = np.array([[0, 0, -1],[-np.cos(alfa), -np.sin(alfa), 0]])
	return z_t_i_pred, H_x_pred
	
def associateMeasurement(x_t_pred, P_t_pred, Z_t, R_t, M, g):
	# print('Usao u associate')
	H_t_pred_temp = np.zeros((np.shape(M)[1],2,3)) # H je 2x3 pa imamo za svaku liniju tako
	V = np.zeros((np.shape(M)[1],np.shape(Z_t)[1],2)) # svaka linija iz pocetne mape x svaka linija iz opservacije mape u trenutku x 2(ro i alfa)
	sigma_in_t = np.zeros((np.shape(M)[1],np.shape(Z_t)[1],2,2)) # za jednu liniju je 2x2 
	
	for i in range(np.shape(M)[1]): # prolazimo kroz svaku liniju mape
		[z_t_i_pred, H_x_pred_i] = measurmentFunction(x_t_pred,M[:,i])
		H_t_pred_temp[i,:,:] = H_x_pred_i
		for j in range(np.shape(Z_t)[1]): # prolazimo kroz svaku liniju opservacije mape u trenutku
				
			# Z_t na prvom mestu je ro a na drugom mestu je alfa
			# z_t_i_pred na prvom mestu je alfa - teta a na drugom je ro - nesto
			v_ij = Z_t[:,j].reshape(2,1)-z_t_i_pred;  # ovo ne moze ovako izokrenute su vrednosti 	
			V[i,j,:] = v_ij.transpose() # da bi moglo da se ubaci na mesto
			r_t = np.array(R_t)
			sigma_in_t[i,j,:,:] = np.matmul(np.matmul(H_x_pred_i,P_t_pred),H_x_pred_i.transpose()) + r_t[j,:,:]
	
	v_t_pred = []
	H_t_pred = []
	R_t_pred = []
	
	for i in range(np.shape(V)[0]):
		for j in range(np.shape(V)[1]):
			d_t_ij = np.matmul(np.matmul(V[i,j,:].transpose(),np.linalg.inv(sigma_in_t[i,j,:,:])),V[i,j,:])
			if(d_t_ij < g**2): 
				v_t_pred.append(V[i,j,:])
				H_t_pred.append(H_t_pred_temp[i,:,:])
				R_t_pred.append(r_t[j,:,:])
	# ovo mozda ne treba videti			
	v_t_pred = np.array(v_t_pred,dtype=float)
	H_t_pred = np.array(H_t_pred,dtype=float)
	R_t_pred = np.array(R_t_pred,dtype=float)
	
	return v_t_pred, H_t_pred, R_t_pred	
	
	
	
	
def filterStep(x_t_pred, P_t_pred, v_t_pred, H_t_pred, R_t_pred):
	#print('Usao u filter')
	P_t_pred = P_t_pred.astype(float)
	R_t_pred = block_diag(*R_t_pred) # R=2*kx2*k
	H_t_pred = np.reshape(H_t_pred,(-1,3)) # H=2*kx3
	v_t_pred = np.reshape(v_t_pred,(-1,1)) # V=2*kx1
	
	S = np.matmul(np.matmul(H_t_pred,P_t_pred),H_t_pred.transpose()) + R_t_pred
	K = np.matmul(np.matmul(P_t_pred,H_t_pred.transpose()),np.linalg.inv(S))
	
	x_t = x_t_pred + np.matmul(K,v_t_pred);
	P_t = np.matmul((np.eye(3)-np.matmul(K,H_t_pred)),P_t_pred);
	
	return x_t,P_t
	
def callback_laser(data):
	global prvi_put,M,Z_t,R_t
	Z_temp = []
	R_temp = []
	linije = data.line_segments	
	for i,linija in enumerate(linije):
		Z_temp.append(np.array([linija.angle,linija.radius]))
		cov_mat = np.asarray(linija.covariance)
		R_temp.append(cov_mat.reshape((2,2)))

	Z_t = (np.array(Z_temp)).transpose(); 
	R_t = R_temp;
	if prvi_put:
		M = Z_t;
		prvi_put = False;
	

def callback_odom(data):

	global X_odom,P_odom,u_t,b
	
	cov_mat = data.pose.covariance;
	P_odom_temp = np.array(cov_mat);
	idx = [0,1,5,6,7,11,30,31,35];
	P_odom = P_odom_temp[idx].reshape((3,3));
	position = data.pose.pose.position
	orientation = data.pose.pose.orientation
	# nas kod 
	q_x = data.pose.pose.orientation.x
	q_y = data.pose.pose.orientation.y
	q_z = data.pose.pose.orientation.z
	q_w = data.pose.pose.orientation.w
	teta = math.atan2(2.0*(q_w*q_z+q_x*q_y),1.0-2.0*(q_y*q_y+q_z*q_z))
	# njihov kod
	# _,_,teta = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
	X_odom = np.array([position.x,position.y,teta]).transpose()
	
		
	Ts = 1./30;
	v = data.twist.twist.linear.x;
	w = data.twist.twist.angular.z;
	u_t[0] = u_t[0] + Ts*(v-w*b); # levi tocak
	u_t[1] = u_t[1] + Ts*(v+w*b); # desni tocak	
	
	
	
	
if __name__ == '__main__':	
	

	global prvi_put,M,Z_t,R_t;
	b = 0.16 
	g = 0.1
	
	
	Z_t = []
	R_t = []
	u_t = np.zeros((2,1))
	P_odom = np.empty((3,3))
	X_odom = np.empty((3,1))
	
	rospy.init_node('main',anonymous=False)
	sub1 = rospy.Subscriber("/line_segments",LineSegmentList, callback_laser)
	sub2 = rospy.Subscriber("/odom",Odometry, callback_odom)
	pub = rospy.Publisher('kalman',Float64MultiArray ,queue_size = 1)
	
	# pocetna pozicija
	X = np.array([[0],[0],[0]])
	P = 0.1*np.eye(3) # treba neki mali broj jer je siguran da je u nuli na pocetku

	prvi_put = True;
	M = [];
	while prvi_put:
		t.sleep(1.0)	
	print("Procitana pocetna mapa");
	# print(M)

	while True:
			
		x_t_1 = X
		P_t_pred = P

		[x_t_pred, F_x_pred, F_u_pred] = transitionFunction(x_t_1,u_t,b)
		
		k = 0.5; # ovo menjamo treba da bude malo
		Q_t = np.array([[k*abs(u_t[0]),0],[0,k*abs(u_t[1])]])
		P_t_pred = np.matmul(np.matmul(F_x_pred,P_t_pred),F_x_pred.transpose())+np.matmul(np.matmul(F_u_pred,Q_t),F_u_pred.transpose())

		[v_t_pred, H_t_pred, R_t_pred] = associateMeasurement(x_t_pred, P_t_pred, Z_t, R_t, M, g)
		
		[X,P] = filterStep(x_t_pred, P_t_pred, v_t_pred, H_t_pred, R_t_pred)
		
		#if X[2]>np.pi: # da bi bilo u opsegu -pi do pi, kao sto bi trebalo
		#	pom = np.round(X[2]/(np.pi*2))
		#	X[2]=X[2]-np.pi*2*pom
		#if X[2]<(-np.pi):
		#	pom = np.round(-X[2]/(np.pi*2))
		#	X[2]=X[2]+np.pi*2*pom
			
		rospy.loginfo('Pozicija sa odometry je: ')
		rospy.loginfo(X_odom)
		rospy.loginfo('Pozicija sa Kalmanovim filtrom je: ')
		rospy.loginfo(X)
		
		rospy.loginfo('Kovarijaciona matrica sa odometry je: ')
		rospy.loginfo(P_odom)
		rospy.loginfo('Kovarijaciona matrica sa Kalmanovim filtrom je: ')
		rospy.loginfo(P)
		
		U = np.zeros((2,1)) #treba nam predjeni put za novu periodu
		pub.publish(Float64MultiArray(data = np.array([X[0],X[1],X[2]])))
		t.sleep(0.2)