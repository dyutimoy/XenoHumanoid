import os
import matplotlib.pyplot as plt
import sys
sys.path.append("/home/xenocrypt/Documents/Humanoid/codes/gait_gen/Yinpy")

import chains
import URDF_utils
import plot_utils
import inverse_kinematics

import numpy as np

import math

resources="../resources/2dof.urdf"



Arm=chains.Chain.from_urdf_file(resources)

joint=[0,-1.5707,1.5707,0]
#hoint=[0,1,2,3,0,4,0]

#Arm.plot(joint,ax=None,show=True)

starting_angles_nodes=[0,0,0,0,0]
target_frame=np.eye(4)
target_frame[:3,3]=[2.4,0,0.3]
theta1,theta2 = inverse_kinematics.inverse_kinematics_2dof(Arm,target_frame,starting_angles_nodes)

if theta1 !=None and theta2 !=None :
	joint=[0,theta1,theta2,0]


	Arm.plot(joint,ax=None,show=True)



for i in range(100):


	target_frame[:3,3]=[2*math.cos(i*3.1415/100),0,0.3+2*math.sin(i*3.1415/100)]
	print(i,target_frame[:3,3])
	theta1,theta2 = inverse_kinematics.inverse_kinematics_2dof(Arm,target_frame,starting_angles_nodes)

	if theta1 !=None and theta2 !=None :
		joint=[0,theta1,theta2,0]
		print("-------------------")
		print(target_frame[:3,3])
		print(theta1,theta2)
		print("******************(")

		Arm.plot(joint,ax=None,target=target_frame[:3,3],show=True)9





