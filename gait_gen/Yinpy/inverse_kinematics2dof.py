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

def inverse_kinematics_2dof(chain, target_frame, starting_nodes_angles):

	#only get the postion
	target= target_frame[:3,3]

	if starting_nodes_angles is None:
			raise ValueError("starting_nodes_angles msut be specified")

	#anylatical soln

	a1=chain.links[1].get_transformation_matrix(0)[:3,3]
	l1=chain.links[2].length
	l2=chain.links[3].length

	l3=distance(target,a1)
	#print(a1)
	#print(l1,l2,l3)
	try:
		theta2=round(3.1415-cosine_rule(l1,l2,l3),3)
		#print(theta2)
		theta1=-cosine_rule(l3,l1,l2)
		#print(theta1)
		if((target[2]-a1[2])>0.001):
			
			theta_temp =math.atan((target[2]-a1[2])/target[0])
		else:
			theta_temp=0
		print("Vals :",theta2,theta1,theta_temp)
	except:
		print 1,None,None
		return None,None
	if theta_temp<0:
		theta1=-(3.1415+theta_temp)+theta1

	else:
		theta1=theta1-theta_temp
	print(theta1,theta2)
	#check
	if theta2>1.5707 :
		print("oh",theta2)
		return None,None
	if theta1<-3.1415 :
		print("oooh",theta1)
		return None,None	

	joints=[0,theta1,theta2,0]
	transformation_matrixes = chain.forward_kinematics(joints, full_kinematics=True)	

	(node, rotation) = geometry_utils.from_transformation_matrix(transformation_matrixes[3])

	if(distance(node[:3],target)<0.001):
		print(np.around(node[:3],decimals=2),";;aaaaaaaaa;;;",np.around(target,decimals=2))
		return theta1,theta2 
	else:
		joints=[0,theta1,-theta2,0]
		transformation_matrixes = chain.forward_kinematics(joints, full_kinematics=True)	

		(node, rotation) = geometry_utils.from_transformation_matrix(transformation_matrixes[3])
		if(distance(node[:3],target)<0.001):
			print(np.around(node[:3],decimals=2),";;;;;",np.around(target,decimals=2))
			return theta1,-theta2 
		else:
			print("asdsd")
			return None,None

	#theta2=



			