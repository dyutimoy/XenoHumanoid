import numpy as np

import URDF_utils
#from . import inverse_kinematics as inverse_kinematics
import link as link_lib
import plot_utils

class Chain(object):


	def __init__(self,links, active_links_mask= None, name= "chain", profile=''"",**kwargs):
		self.name=name
		self.links=links
		self._length = sum([link.length for link in links])

		for (index, link) in enumerate(self.links):
			if link.length  == 0:
				link.axis_length  = self.links[index -1].axis_length

		if active_links_mask is not None:
			if len(active_links_mask) !=len(self.links):
				raise ValueError("Your active links mask length of  {} is different from the number of your links, which is {}". format(len(active_links_mask),len(self.links)))	

			self.active_links_mask=np.array(active_links_mask)

			self.active_links_mask[-1] =False 
		else:
			self.active_links_mask = np.array([True]*len(links))


	def __repr__(self):
		return "kinematic chain name={} links={} active_links={}".format(self.name, self.links,self.active_links_mask)		


	def forward_kinematics(self, joints, full_kinematics= False):
		
		frame_matrix = np.eye(4)

		if full_kinematics:
			frame_matrixes = []

		

		if len(self.links) != len(joints):
			raise ValueError("Your joints vector length is {} but you have {} links".format(len(joints), len(self.links)))	

		for index, (link,joint_angle) in enumerate(zip(self.links,joints)):
			
			#print(np.around(np.asarray(link.get_transformation_matrix(joint_angle)),decimals=2))
			#print(index, np.around(np.asarray(link.get_transformation_matrix(joint_angle)), decimals=2))
			#if(index>0):
			#	print("index",index)
			#	print(link.sym_transformation_matrix)
			frame_matrix=np.dot(frame_matrix, np.asarray(link.get_transformation_matrix(joint_angle)))
				
			if full_kinematics:
				frame_matrixes.append(frame_matrix)
			#print(np.around(frame_matrix,decimals=2))
			#print(index)

		if full_kinematics:
			return frame_matrixes
		else:
			return frame_matrix			

	def plot(self, joints, ax, target=None, show=False):

		if ax is None:
			# If ax is not given, create one
			ax = plot_utils.init_3d_figure()
		plot_utils.plot_chain(self, joints, ax)
		plot_utils.plot_basis(ax, self._length)

		# Plot the goal position
		if target is not None:
			plot_utils.plot_target(target, ax)
		if show:
			plot_utils.show_figure()
			

	@classmethod
	def from_urdf_file(cls, urdf_file, base_elements=None, last_link_vector=None, base_element_type="link", active_links_mask=None, name="chain"):

		if base_elements is None:
			base_elements = ["base_link"]

		links = URDF_utils.get_urdf_parameters(urdf_file, base_elements=base_elements, last_link_vector=last_link_vector, base_element_type=base_element_type)
		# Add an origin link at the beginning
		return cls([link_lib.OriginLink()] + links, active_links_mask=active_links_mask, name=name)

	def active_to_full(self, active_joints, initial_position):
		full_joints = np.array(initial_position, copy=True, dtype=np.float)
		np.place(full_joints, self.active_links_mask, active_joints)
		return full_joints

	def active_from_full(self, joints):
		return np.compress(self.active_links_mask, joints, axis=0)

	@classmethod
	def concat(cls, chain1, chain2):
		return cls(links=chain1.links + chain2.links, active_links_mask=chain1.active_links_mask + chain2.active_links_mask)
