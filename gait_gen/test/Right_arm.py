import os
import matplotlib.pyplot as plt
import sys
sys.path.append("/home/xenocrypt/Documents/Humanoid/codes/gait_gen/Yinpy")

import chains
import URDF_utils
import plot_utils
import ik_rightarm
import geometry_utils

import numpy as np

import math
import random
resources="../resources/10-armchain.urdf"


Arm=chains.Chain.from_urdf_file(resources)

joint=[0,0,0,0,0,0,0]
#joint=[0,1,2,3,0,4,0]



starting_angles_nodes=[0,np.pi/4,np.pi/2,-np.pi/4,0, np.pi/4 ,0]
target_frame = Arm.forward_kinematics(starting_angles_nodes, full_kinematics=False)	
#Arm.plot(starting_angles_nodes,ax=None,show=True)
#target_frame=np.array([[ -1. ,  0. ,  0. ,  0. ],
#      		  [ 0. ,  0. ,  -1. , -0.6],
#      		  [ 0. ,  -1. ,  0. ,  1.4],
#      		  [ 0. ,  0. ,  0. ,  1. ]])
#target=[0.424, 0.624 ,0.0]


#Arm.plot(joints=starting_angles_nodes,ax=None,target=target,show=True)
All_sol=ik_rightarm.inverse_kinematics(Arm,target_frame,starting_angles_nodes)
print(All_sol)
#check using forward kinematics

for i in range(2):
	print("*************************************************************")
	theta1=random.randrange(-135,45,1)
	theta2=random.randrange(-90,90,1)
	theta3=random.randrange(-90,0,1)
	theta4=random.randrange(0,180,1)


	print(theta1,theta2,theta3,theta4)

	starting_angles_nodes=[0,theta1*(np.pi/180.0),theta2*(np.pi/180.0),theta3*(np.pi/180.0),0, theta4*(np.pi/180.0) ,0]
	print("starting_angles_nodes]",starting_angles_nodes)
	target_frame = Arm.forward_kinematics(starting_angles_nodes, full_kinematics=False)

	All_sol=ik_rightarm.inverse_kinematics(Arm,target_frame,starting_angles_nodes)
	
	for j in range(All_sol.shape[0]):
		flag=1		
		for k in range(4):
			if(All_sol[j,k]<-50):
				flag=0

		if(flag==1):
			result_angles=[0,All_sol[j,0],All_sol[j,1],All_sol[j,2],0, All_sol[j,3] ,0]
			
			target_result = Arm.forward_kinematics(starting_angles_nodes, full_kinematics=False)
			if(np.linalg.norm(target_frame[:3,3]-target_result[:3,3])<0.01)
				print("result_angles",result_angles)
				Arm.plot(joints=result_angles,ax=None,target=target_result,show=True)
				break
			
			
			
			
			

	