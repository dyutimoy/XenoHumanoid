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
	#print(a1)		
	l1=chain.links[5].length
	#print(l1)
	l2=chain.links[6].length
	#print(l2)

	theta_all=np.ones((2,4))*-100
	print(l0,l1,l2)

	#theta4
	a6=chain.links[6].get_transformation_matrix(0)[:3,3]
	#print(a6)
	l3=distance(target,a1)	
	print("hhi")
	try:	

		theta4_=cosine_rule(l1,l2,l3)
		#As theta4 can take only postive values,theta4=3.1415-theta4_
		print("theta4_",theta4_)
		theta4=3.1416-theta4_
		print("theta4",theta4)
		#theta2
		for i in range(4):
			theta_all[i,3]=theta4
		print(np.round(theta_all,decimals=3))	
		try:	
			print("Ssd")
			print(target_frame[1,3],l0,l2,target_frame[1,1],l1)
			theta2=math.asin((target_frame[1,3]+l0-l2*target_frame[1,1])/(l1))
			print(theta2)

			
			theta_all[0,1]=theta2
			theta_all[1,1]=theta2


			try:
				if(abs(math.cos(theta2))>0.001):
					theta3=math.acos(-target_orient[1,2]/math.cos(theta2))
					print("theta3",theta3)
					if theta3<=np.pi/2:
						theta3=-theta3
						theta_all[0,2]=theta3
						theta_all[1,2]=theta3

						#theta1 solve simultaneous equations
						try:
							if((abs(math.sin(theta2))<0.0001 or abs(math.cos(theta3))<0.0001) and abs(math.sin(theta3))>0.0001):
								theta1=math.asin (target_frame[2,2]/math.sin(theta3))
							elif(abs(math.sin(theta2))>0.0001  and abs(math.sin(theta3))<0.0001):
								theta1=math.asin(-target_frame[0,2]/(math.sin(theta2)))
							elif(abs(math.sin(theta2))<0.0001  and abs(math.sin(theta3))<0.0001 and abs(theta4-np.pi/4)>0.001):
								theta1=math.asin(-target_frame[0,0]/(math.cos(theta4)+math.sin(theta4)))									
							elif(abs(math.sin(theta2))<0.0001  and abs(math.sin(theta3))<0.0001 and abs(theta4-np.pi/4)>0.001):
								theta1=math.asin(-target_frame[0,1]*math.cos(theta4)-target_frame[2,1]*math.sin(theta4))
							elif((target_frame[2,2]*math.sin(theta3)-target_frame[0,2]*math.sin(theta2)*math.cos(theta3))< 0.0001):
								theta1=math.asin(0)

							else:
								theta1=math.asin((target_frame[2,2]*math.sin(theta3)-target_frame[0,2]*math.sin(theta2)*math.cos(theta3))/(math.sin(theta3)**2+(math.sin(theta2)*math.cos(theta3))**2))
								print("theta1",theta1)
								
							if theta1>=-np.pi/2 and theta1<=np.pi/4:
								theta_all[0,0]=theta1
							if(theta1<=-np.pi/4):
								theta_all[1,0]=-np.pi-theta1
						except:
							print("theta val not defined case 1,2 theta1")
							return theta_all
				else:
					if(math.cos(theta4)>0.001):
						theta_sum=math.asin(target_orient[2,0]/math.cos(theta4))
						flag=1
						theta3=starting_nodes_angles[3]
						count=0
						while flag:
							
							theta3=starting_nodes_angles[3]+(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=theta_sum-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0
									break

							theta3_=starting_nodes_angles[3]+(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=-(theta_sum+np.pi/2)-np.pi/2-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0	
									break

									
							theta3=starting_nodes_angles[3]-2*(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=theta_sum-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0
									break	

							theta3_=starting_nodes_angles[3]-2*(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=-(theta_sum+np.pi/2)-np.pi/2-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0
									break	
		



							count+=0.1
							if(count>90):
								flag=0


								
					elif(math.sin(theta4)>0.001):
						theta_sum=math.asin(-target_orient[2,1]/math.sin(theta4))
						flag=1
						theta3=starting_nodes_angles[3]
						count=0.0
						while flag:
							

							theta3=starting_nodes_angles[3]+(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=theta_sum-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0
									break

							theta3_=starting_nodes_angles[3]+(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=-(theta_sum+np.pi/2)-np.pi/2-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0	
									break

									
							theta3=starting_nodes_angles[3]-2*(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=theta_sum-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0
									break	

							theta3_=starting_nodes_angles[3]-2*(count*np.pi/180.0)
							if(theta3>-np.pi/2 and theta3<0):
								theta1=-(theta_sum+np.pi/2)-np.pi/2-theta3
								if(theta1>-np.pi*3/4 and theta1<np.pi/4):
									theta_all[0,2]=theta3
									theta_all[1,2]=theta3
									theta_all[0,0]=theta1
									theta_all[1,0]=theta1
									flag=0	
									break

							count+=0.1
							if(count>90):
								flag=0

			


			
			except:
				print("theta val not defined case 1,2 theta3")
				return theta_all			
		except:
			print("theta val not defined case 1,2 theta2")
			return theta_all
	except:
		print("theta val not defined case 1,2 theta4")	
		return theta_all			

	return theta_all			


				
