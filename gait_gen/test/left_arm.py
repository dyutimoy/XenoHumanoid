import os
import matplotlib.pyplot as plt
import sys
sys.path.append("/home/xenocrypt/Documents/Humanoid/codes/gait_gen/Yinpy")

import chains
import URDF_utils
import plot_utils
import ik_leftarm
import geometry_utils

import numpy as np

import math

resources="../resources/10-armchain.urdf"


Arm=chains.Chain.from_urdf_file(resources)

joint=[0,0,0,0,0,0,0]
#joint=[0,1,2,3,0,4,0]



starting_angles_nodes=[0,0,0,0,0, np.pi/2 ,0]
target_frame = Arm.forward_kinematics(starting_angles_nodes, full_kinematics=False)	
#Arm.plot(starting_angles_nodes,ax=None,show=True)
#target_frame=np.array([[ -1. ,  0. ,  0. ,  0. ],
#      		  [ 0. ,  0. ,  -1. , -0.6],
#      		  [ 0. ,  -1. ,  0. ,  1.4],
#      		  [ 0. ,  0. ,  0. ,  1. ]])
#target_frame[:3,3]=[0,-0.6,1.4]

All_sol=ik_leftarm.inverse_kinematics(Arm,target_frame,starting_angles_nodes)
print(All_sol)
#check using forward kinematics
"""
opt_theta=[0,0,0,0,0,0,0]
max_disp=999999999
for i in range(All_sol.shape[0]):
	flag=0

	for  j in range(4):
		if All_sol[i,j]<-50  :
			flag=1

	print(flag,i)		
	if flag==0:
		angle_vals=[0,All_sol[i,0],All_sol[i,1],All_sol[i,2],0,All_sol[i,3],0]
		transformation_matrix = Arm.forward_kinematics(angle_vals, full_kinematics=False)	
		print(transformation_matrix)
		#(node, rotation) = geometry_utils.from_transformation_matrix(transformation_matrix)

		if(ik_leftarm.distance(transformation_matrix,target_frame)<0.01 and ik_leftarm.distance(transformation_matrix,target)<max_disp):
			print(np.around(transformation_matrix,decimals=2),";;aaaaaaaaa;;;",np.around(target,decimals=2))
			max_disp=ik_leftarm.distance(transformation_matrix,target)
			print(max_disp)
			opt_theta=angle_vals

if max_disp !=999999999:
	Arm.plot(opt_theta,ax=None,target=target_frame[:3,3],show=True)

else:
	print("no sol")	

"""