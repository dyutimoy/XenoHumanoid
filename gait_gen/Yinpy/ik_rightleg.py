import scipy.optimize
import numpy as np
import os

import math
import chains
import chains
import URDF_utils
import plot_utils
import geometry_utils

def cosine_rule(l_1,l_2,l_3):
	
	theta=math.acos((l_1**2 + l_2**2-l_3**2)/(2*l_1*l_2))
	return round(theta,4)
	
def distance(p1,p2):
	return np.linalg.norm(p1-p2)

def inverse_kinematics(chain, target_frame, starting_nodes_angles):

	#only get the postion
	

	target= target_frame[:3,3]
	target_orient=target_frame[:3,:3]
	print("target",np.round(target_frame,decimals=3))
	if starting_nodes_angles is None:
			raise ValueError("starting_nodes_angles msut be specified")


	a1=chain.links[1].get_transformation_matrix(0)[:3,3]
	l0=chain.links[1].length
	print(a1)		
	l1=chain.links[4].length
	print(l1)
	l2=chain.links[5].length
	print(l2)

	theta_all=np.ones((2,3))*-100
	#print(l0,l1,l2)
	#print("starting_nodes_angles",starting_nodes_angles)

	#theta3
	a5=chain.links[5].get_transformation_matrix(0)[:3,3]
	#print(a6)
	l3=distance(target,a1)	
	#print("hhi")
	try:	

		theta3_=cosine_rule(l1,l2,l3)

		#As theta4 can take only postive values,theta4=3.1415-theta4_
		#print("theta4_",theta4_)
		theta3=-(3.1416-theta3_)
		print(theta3)
		if(theta3>=-np.pi/2):
			print("theta4",theta3)
			#theta2
			for i in range(2):
				theta_all[i,2]=theta3
		else:
			print(" out of range theta3")
			return theta_all		
		#print(np.round(theta_all,decimals=3))	
		try:	
			#print("Ssd")
			#print(target_frame[1,3],l0,l2,target_frame[1,1],l1)
			theta1=math.asin(-target_frame[2,2])
			

			if(theta1>=-np.pi/4 and theta1<=np.pi/4):
				theta_all[0,0]=theta1
				theta_all[1,0]=theta1
				print(theta1)
			else:	
				print(" out of range theta1")
				return theta_all

			try:
				theta_sum=math.asin(target_frame[0,0])
				if(theta_sum>-np.pi/4):
					theta2=theta_sum-theta3
					if(theta2>=-np.pi/4 and theta2<=np.pi/2):
						theta_all[0,1]=theta2
						theta_all[1,1]=theta2
						print(theta2)
					else:	
						print(" out of range theta2")
						return theta_all	
				else:
					theta2=theta_sum-theta3
					if(theta2>=-np.pi/4 and theta2<=np.pi/2):
						theta_all[0,1]=theta2
						print(theta2)
					theta2=-np.pi-theta_sum-theta3
					if(theta2>=-np.pi/4 and theta2<=np.pi/2):
						theta_all[1,1]=theta2
						print(theta2)

			except:
				print("theta val not defined case 1,2 theta2")
				return theta_all			
		except:
			print("theta val not defined case 1,2 theta1")
			return theta_all
	except:
		print("theta val not defined case 1,2 theta3")	
		return theta_all			

	return theta_all			


				
