
import numpy as np
import sympy



def Rx_matrix(theta):

	return np.array([
		[1,0,0],
		[0,np.cos(theta),-np.sin(theta)],
		[0,np.sin(theta),np.cos(theta)]
	])


def Rz_matrix(theta):
		
		return np.array([
			[np.cos(theta),-np.sin(theta),0],
			[np.sin(theta),np.cos(theta),0],
			[0,0,1]
		])

def symbolic_Rz_matrix(symbolic_theta):
	#theta about z is only variable
	return sympy.Matrix([
        [sympy.cos(symbolic_theta), -sympy.sin(symbolic_theta), 0],
        [sympy.sin(symbolic_theta), sympy.cos(symbolic_theta), 0],
        [0, 0, 1]
    ])

def Ry_matrix(theta):
	
	return np.array([
		[np.cos(theta),0,np.sin(theta)],
		[0,1,0],
		[-np.sin(theta),0,np.cos(theta)]
	])



def rotation_matrix(phi, theta, psi):

	return np.dot(Rz_matrix(phi), np.dot(Rx_matrix(theta), Rz_matrix(psi)))


def symbolic_rotation_matrix(phi, theta, symbolic_psi):
    
	return sympy.Matrix(Rz_matrix(phi)) * sympy.Matrix(Rx_matrix(theta)) * symbolic_Rz_matrix(symbolic_psi)


def rpy_matrix(roll, pitch, yaw):
	#print("ss",roll,pitch,yaw)
	#print(np.dot(Rz_matrix(yaw), np.dot(Ry_matrix(pitch), Rx_matrix(roll))))
	return np.dot(Rz_matrix(yaw), np.dot(Ry_matrix(pitch), Rx_matrix(roll)))


def symbolic_axis_rotation_matrix(axis,symbolic_theta):
	
	[x,y,z]=axis
	c=sympy.cos(symbolic_theta)
	s=sympy.sin(symbolic_theta)

	return sympy.Matrix([
		[x**2 + (1 - x**2) * c, x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [x * y * (1 - c) + z * s, y ** 2 + (1 - y**2) * c, y * z * (1 - c) - x * s],
        [x * z * (1 - c) - y * s, y * z * (1 - c) + x * s, z**2 + (1 - z**2) * c]
	])    

def homogeneous_translation_matrix(trans_x,trans_y,trans_z):

	return np.array([[1,0,0,trans_x],[0,1,0,trans_y],[0,0,1,trans_z],[0,0,0,1]])


def from_transformation_matrix(transformation_matrix):
	return (transformation_matrix[:, -1], transformation_matrix[:-1, :-1])                          #change this if required translantion is wrong


def to_transformation_matrix(translation, orientation_matrix=np.zeros((3, 3))):
	matrix = np.eye(4)

	matrix[:-1, :-1] = orientation_matrix
	matrix[:-1, -1] = translation
	return matrix



	
def cartesian_to_homogeneous(cartesian_matrix, matrix_type="numpy"):
	"""Converts a cartesian matrix to an homogenous matrix"""
	dimension_x, dimension_y = cartesian_matrix.shape
	# Square matrix
	# Manage different types fo input matrixes
	if matrix_type == "numpy":
		homogeneous_matrix = np.eye(dimension_x + 1)
	elif matrix_type == "sympy":
		homogeneous_matrix = sympy.eye(dimension_x + 1)
	# Add a column filled with 0 and finishing with 1 to the cartesian matrix to transform it into an homogeneous one
	homogeneous_matrix[:-1, :-1] = cartesian_matrix

	return homogeneous_matrix


def cartesian_to_homogeneous_vectors(cartesian_vector, matrix_type="numpy"):
	"""Converts a cartesian vector to an homogenous vector"""
	dimension_x = cartesian_vector.shape[0]
	# Vector
	if matrix_type == "numpy":
		homogeneous_vector = np.zeros(dimension_x + 1)
		# Last item is a 1
		homogeneous_vector[-1] = 1
		homogeneous_vector[:-1] = cartesian_vector
	return homogeneous_vector


def homogeneous_to_cartesian_vectors(homogeneous_vector):
	"""Converts a cartesian vector to an homogenous vector"""
	return homogeneous_vector[:-1]


def homogeneous_to_cartesian(homogeneous_matrix):
	"""Converts a cartesian vector to an homogenous matrix"""
	# Remove the last column
	return homogeneous_matrix[:-1, :-1]


