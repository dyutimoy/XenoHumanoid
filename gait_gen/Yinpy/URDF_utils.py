



import xml.etree.ElementTree as ET
import json
import numpy as np
import itertools

import link as lib_link
#rom . import logs

def find_next_joint(root,current_link):

	has_next=False
	next_joint = None
	search_by_name = True
	current_link_name = None


	current_link_name=current_link.attrib["name"]


	for joint in root.findall("joint"):

		if joint is not None:

			if joint.find("parent").attrib["link"] == current_link_name:
				has_next = True
				next_joint =joint
				break		


	return has_next, next_joint
	

def find_next_link(root,current_joint):
	
	has_next = False
	next_link= None


	

	next_link_name=current_joint.find("child").attrib["link"]

	for urdf_link in root.findall("link"):
		if urdf_link.attrib["name"]  == next_link_name:
			next_link =urdf_link
			has_next =True

	return has_next,next_link			


def parent_link(root, joint_name):
	return next(joint.find("parent").attrib["link"]
				for joint in root.iter("joint")
				if joint.attrib["name"]==joint_name)


def get_chain_from_joints(urdf_file, joints):


	tree = ET.parse(urdf_file)
	root = tree.getroot()

	links =[ find_parent_link(root,j) for j in joints]

	iters = [iter(links), iter(joints)]

	chain = list(next(it) for it in itertools.cycle(iters))

	return chain


def get_urdf_parameters(urdf_file, base_elements= None, last_link_vector= None, base_element_type="link"):


	tree = ET.parse(urdf_file)
	root = tree.getroot()
	base_elements = list(base_elements)
	if base_elements is None:
		base_elements = ["base_link"]
	elif base_elements is  []:
		raise ValueError("base_elements can't be the empty list []")


	joints = []
	links = []
	has_next = True
	current_joint = None
	current_link = None

	if base_element_type == "link":
		node_type = "link"
	elif base_element_type == "joint":
		node_type="joint"
	else:
		raise ValueError("Unknown type: {}".format(base_element_type))


	next_link_name=base_elements.pop(0)
	for urdf_link in root.findall("link"):
		if urdf_link.attrib["name"]  == next_link_name:
			current_link =urdf_link

	while has_next:
		if node_type == "link":
			
			(has_next, current_joint) = find_next_joint(root, current_link)
			node_type="joint"

			if has_next:
				joints.append(current_joint)

		elif node_type =="joint":
			(has_next, current_link) = find_next_link(root, current_joint)
			node_type = "link"	
			if has_next:
				links.append(current_link)

	parameters = []
	

	for joint in joints:
		translation = [0,0,0]
		orientation = [0,0,0]
		rotation =[0,0,1]
		bounds = [None,None]


		origin = joint.find("origin")
		if origin is not None:
			if origin.attrib['xyz']:
				translation =[float(x) for x in origin.attrib["xyz"].split()]
			if origin.attrib["rpy"]:
				orientation = [float(x) for x in origin.attrib["rpy"].split()]

		axis = joint.find("axis")
		if axis is not None:
			rotation = [float(x) for x in axis.attrib["xyz"].split()]



		limit = joint.find("limit")
		if limit is not None:
			if limit.attrib["lower"]:
				bounds[0]=float(limit.attrib["lower"])
			if limit.attrib["upper"]:
				bounds[1] = float(limit.attrib["upper"])

		parameters.append(lib_link.URDFLink(
			name=joint.attrib["name"],
			bounds=tuple(bounds),
			translation_vector=translation,
			orientation=orientation,
			rotation=rotation
		))					
		#print(orientation)

	if last_link_vector is not None:
		parameters.append(lib_link.URDFLink(
			translation_vector=last_link_vector,
			orientation=[0,0,0],
			rotation=[1,0,0],
			name="last_joint"
		))

	return parameters

