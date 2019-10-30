#!/usr/bin/env python
# coding: utf-8

# In[1]:


import sympy as sp
import numpy as np





from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)



from sympy.physics.mechanics import dynamicsymbols




theta0,theta1, theta2,theta3,theta4,Ch,A_r,B_r,H, theta, alpha, a, d = dynamicsymbols('theta0 theta1 theta2 theta3 theta4  Ch A_r B_r H theta alpha a d')




rot = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha)],
                 [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha)],
                 [0, sp.sin(alpha), sp.cos(alpha)]])

trans = sp.Matrix([a*sp.cos(theta),a*sp.sin(theta),d])

last_row = sp.Matrix([[0, 0, 0, 1]])

m = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)




t_0=sp.pi
t_1=-sp.pi/2
t_2=sp.pi/2
t_3=-sp.pi/2
t_4=0


# In[7]:


m00=sp.Matrix([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1]])
m_00=m00




m00_= sp.Matrix([[1,0,0,0],
                 [0,1,0,-Ch],
                 [0,0,1,0],
                 [0,0,0,1]])
m0_1 = m.subs({alpha:sp.pi/2, a:0, theta:t_0+theta0, d:-0})
m01=m00_*m0_1



t0=sp.pi
m_01=m01.subs({theta0:t0})


m12 = m.subs({alpha:sp.pi/2, a:0, theta:t_1+theta1, d:0})



m23 = m.subs({alpha:sp.pi/2, a:0, theta:t_2+theta2, d:0})


m34 = m.subs({alpha:sp.pi/2, a:0, theta:t_3+theta3,d:A_r})



m44_=m.subs({alpha:0, a:0, theta:t_4+theta4,d:0})

m4a_= sp.Matrix([[1,0,0,0],
                 [0,1,0,B_r],
                 [0,0,1,0],
                 [0,0,0,1]])
m4a=m44_*m4a_


# In[21]:

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 

import numpy as np
import time
arm_length=1.6
workspace=np.zeros((10000))

for i in range(10):
    t0=0
    t1=(i/180.0)*(np.pi)*9.0
    for j in range(10):
        t2=(j/180.0)*(np.pi)*9.0
        for k in range(10):
            t3=(k/180.0)*(np.pi)*9.0
            for l in range(10):
            	
                t4=(l/180.0)*(np.pi)*9.0
                print(t1,t2,t3,t4)

                c=0.6
                ar=0.8
                br=0.6
                h=5
                
                m_01=m01.subs({theta0:t0,theta1:t1,theta2:t2, theta3:t3,theta4:t4,Ch:c,A_r:ar, B_r:br,H:h})
                
                m02=m01*m12
                m_02=m02.subs({theta0:t0,theta1:t1,theta2:t2, theta3:t3,theta4:t4,Ch:c,A_r:ar, B_r:br,H:h})
                
                m03=m01*m12*m23
                m_03=m03.subs({theta0:t0,theta1:t1,theta2:t2, theta3:t3,theta4:t4,Ch:c,A_r:ar, B_r:br,H:h})
                

                m04=m01*m12*m23*m34
                m_04=m04.subs({theta0:t0,theta1:t1,theta2:t2, theta3:t3,theta4:t4,Ch:c,A_r:ar, B_r:br,H:h})
                

                m0a=m01*m12*m23*m34*m4a
                m_0a=m0a.subs({theta0:t0,theta1:t1,theta2:t2, theta3:t3,theta4:t4,Ch:c,A_r:ar, B_r:br,H:h})
                


                fig = plt.figure()

                
                ax = fig.add_subplot(111, projection='3d')
                ax.view_init(azim=-90, elev=0)
                ax = plt.axes(projection='3d')
                ax.set_xlim3d([-3.0, 3.0])
                ax.set_xlabel('X')

                ax.set_ylim3d([-3.0, 3.0])
                ax.set_ylabel('Y')

                ax.set_zlim3d([-3.0, 3.0])
                ax.set_zlabel('Z')

    
                ax.plot([0, arm_length * 1.5], [0, 0], [0, 0])
                ax.plot([0, 0], [0, arm_length * 1.5], [0, 0])
                ax.plot([0, 0], [0, 0], [0, arm_length * 1.5])

				

                x = [m_00[0,3],m_01[0,3],m_02[0,3],m_03[0,3],m_04[0,3],m_0a[0,3]]
                y = [m_00[1,3],m_01[1,3],m_02[1,3],m_03[1,3],m_04[1,3],m_0a[1,3]]
                z = [m_00[2,3],m_01[2,3],m_02[2,3],m_03[2,3],m_04[2,3],m_0a[2,3]]
                plt.plot(y,z)

                workspace.append(x,y,z)
              	#print("print",x)
              	#print("printy",y)
              	#print("z",z)
                plt.show(block=False)
                plt.pause(.1)
                plt.close()
				
				               

# In[15]:


