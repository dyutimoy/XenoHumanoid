import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d


x=[]
y=[]
z=[]
sum=-225
for i in range(1801):
	for j in range(901):
		theta1=-135+0.1*i
		theta2=-90+0.1*j
		
		x.append(theta1)
		y.append(theta2)
		z.append(theta1+theta2)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x,y,z)
ax.set_xlabel('xaxis')
ax.set_ylabel('yaxis')
ax.set_zlabel('z-axis')	

plt.show()		