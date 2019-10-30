import numpy as np
import sympy
import geometry_utils

class Link(object):



	def __init__(self,name,length, bounds=(None,None)):

		self.bounds=bounds
		self.name=name
		self.length= length
		self.axis_length=length

	def  __repr__(self):
		return "Link name={} bounds={}".format(self.name,self.bounds)	

	def  _get_rotation_axis(self):
		return [0,0,0,1]

	def get_transformation_matrix(self,theta):
		raise NotImplementedError


class URDFLink(Link):

	#rotation ??

	def __init__(self,name,translation_vector,orientation,rotation, bounds=(None,None), angle_representation='rpy',use_symbolic_matrix=True):
		Link.__init__(self,name=name,bounds=bounds,length=np.linalg.norm(translation_vector))
		self.use_symbolic_matrix=use_symbolic_matrix
		self.translation_vector=np.array(translation_vector)
		self.orientation=np.array(orientation)
		self.rotation=np.array(rotation)


		if use_symbolic_matrix:

			theta = sympy.symbols("theta")

			symbolic_frame_matrix =np.eye(4)

			symbolic_frame_matrix = symbolic_frame_matrix * sympy.Matrix(geometry_utils.homogeneous_translation_matrix(*translation_vector))
			#sympy conv
			symbolic_frame_matrix=symbolic_frame_matrix*geometry_utils.cartesian_to_homogeneous(geometry_utils.rpy_matrix(*orientation))
			#print("yo",symbolic_frame_matrix)
			symbolic_frame_matrix = symbolic_frame_matrix * geometry_utils.cartesian_to_homogeneous(geometry_utils.symbolic_axis_rotation_matrix(rotation, theta), matrix_type="sympy")
			#print("hsh",theta,symbolic_frame_matrix)
			self.sym_transformation_matrix=symbolic_frame_matrix
			self.symbolic_transformation_matrix=sympy.lambdify(theta,symbolic_frame_matrix,"numpy")


	def __str__(self):
		return("""URDF Link {} :
    Bounds : {}
    Translation : {}
    Orientation : {}
    Rotation : {}""".format(self.name, self.bounds, self.translation_vector, self.orientation, self.rotation))	


	def  _get_rotation_axis(self):
		return np.dot(
			geometry_utils.homogeneous_translation_matrix(*self.translation_vector),
			np.dot(
				geometry_utils.cartesian_to_homogeneous(geometry_utils.rpy_matrix(*self.orientation)),
				geometry_utils.cartesian_to_homogeneous_vectors(self.rotation*self.axis_length)
			)
		)	

	def get_transformation_matrix(self,theta):
		if self.use_symbolic_matrix:
			frame_matrix = self.symbolic_transformation_matrix(theta)
			#print(np.around(frame_matrix,decimals=2))

		else:
			frame_matrix=np.eye(4)

			frame_matrix = np.dot(frame_matrix, geometry_utils.homogeneous_translation_matrix(*self.translation_vector))	

			frame_matrix = np.dot(frame_matrix, geometry_utils.cartesian_to_homogeneous(geometry_utils.rpy_matrix(*self.orientation)))

			# Apply rotation matrix
			frame_matrix = np.dot(frame_matrix, geometry_utils.cartesian_to_homogeneous(geometry_utils.axis_rotation_matrix(self.rotation, theta)))

		return frame_matrix


class DHLink(Link):


	def __init__(self, name, d=0,a=0, bounds=None, use_symbolic_matrix=True):
		Link.__init__(self, use_symbolic_matrix)


	def get_transformation_matrix(self,theta,a):
	
		ct = np.cos(theta + self.theta)
		st = np.sin(theta + self.theta)
		ca = np.cos(self.alpha)
		sa = np.sin(self.alpha)

		return np.matrix(((ct, -st * ca, st * sa, a * ct),
						  (st, ct * ca, -ct * sa, a * st),
                          (0, sa, ca, self.d),
                          (0, 0, 0, 1)))

class OriginLink(Link):

	def __init__(self):
		Link.__init__(self,name="Base link", length=1)

	def _get_rotation_axis(self):
		return [0,0,0,1]

	def get_transformation_matrix(self, theta):
		return np.eye(4)


