#!/usr/bin/env python
import rospy
import numpy as np
import sys
import sensor_msgs
import struct
import random
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
import tf

# Definition of class
class Estimation_Node:
	def __init__(self, node_name):
		self.nname = node_name 	#Giving a name for the ROS node

		rospy.init_node(self.nname, anonymous=True) #ROS node initialization

		self.num_of_plane_points = 100 # This sets a minimum number of points used to estimate a 3D plane

		self.plane_params = {"red":[-1.0]*4, "green":[-1.0]*4, "blue":[-1.0]*4} # A dictionnary holding the plane parameters, 4 per plane equation ax+by+cz+d = 0

		self.plane_points = {"red":[], "green":[], "blue":[]}	# dico for the plane points

		self.feature_pose = Transform(Vector3(0, 0, 0.5), tf.transformations.quaternion_from_euler(0, 0, 0)) # This will hold the 6DOF pose of the feature, by a 3D vector for the translation and a quaternion for the rotation

		self.linear_solution = [] # This will hold the point of planes intersection obtained by solving a 3x3 linear system of equations
	
		point_cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.estimate_pose_callback) # ROS topic subscription

		self.br = tf.TransformBroadcaster()

		rospy.spin() # Initiate the ROS loop

	def empty_points(self):
		self.plane_points["red"] = []
		self.plane_points["green"] = []
		self.plane_points["blue"] = []
		
	def estimate_pose_callback(self, pointcloud_msg):
		#print 'Received PointCloud2 message. Reading data...'
		point_list = sensor_msgs.point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names = ("x", "y", "z", "rgb"))

		#print 'Retrieving coordinates and colors...'
		for point in point_list:
			rgb = struct.unpack('BBBB', struct.pack('f', point[3]))

			if rgb[2] > 100 and rgb[0] < 20 and rgb[1] < 20: # If dominant red point, concatenate it
				self.plane_points["red"] += [[point[0], point[1], point[2]]]
			elif rgb[1] > 100 and rgb[0] < 20 and rgb[2] < 20: # If dominant green point, concatenate it
				self.plane_points["green"] += [[point[0], point[1], point[2]]]
			elif rgb[0] > 100 and rgb[2] < 20 and rgb[1] < 20: # If dominant blue point, concatenate it
				self.plane_points["blue"] += [[point[0], point[1], point[2]]]


		# boolean to check if there are enough points or if the plans are orthogonal
		enoughPoints = True
		orthogonalPlanes = True

		# Test if there are sufficient points for each plane
		if len(self.plane_points["red"]) < self.num_of_plane_points:
			print("Not enough points of red plane")
			enoughPoints = False
		elif len(self.plane_points["green"]) < self.num_of_plane_points:
			print("Not enough points of green plane")
			enoughPoints = False
		elif len(self.plane_points["blue"]) < self.num_of_plane_points:
			print("Not enough points of blue plane")
			enoughPoints = False
		
		if (enoughPoints):
			# get only for points for every plane
			selected_red_points = list()
			for i in range(4):
				selected_red_points.append(random.choice(self.plane_points["red"]))
			selected_green_points = list()
			for j in range(4):
				selected_green_points.append(random.choice(self.plane_points["green"]))
			selected_blue_points = list()
			for k in range(4):
				selected_blue_points.append(random.choice(self.plane_points["blue"]))
				
			#selected_red_points = random.choices(plane_points["red"], k=4)
			#selected_green_points = random.choices(plane_points["green"], k=4)
			#selected_blue_points = random.choices(plane_points["blue"], k=4)
		
		
			# Estimate the plane equation for each colored point set using Least Squares algorithm
			# To estimate every plan, we set d = -1. We obtain the following equations :
			B = [1.0, 1.0, 1.0, 1.0]
			
			
			"""
			print(type(self.plane_params["red"][0]))
			print(type(B[0]))
			print(len(selected_red_points))
			print(selected_red_points)
			print(type(selected_red_points[0]))
			print(type(np.linalg.lstsq(selected_red_points, B, rcond=None)[0]))
			"""
			
			self.plane_params["red"][0], self.plane_params["red"][1], self.plane_params["red"][2] = np.linalg.lstsq(selected_red_points, B, rcond=1.e-10)[0]
			self.plane_params["green"][0], self.plane_params["green"][1], self.plane_params["green"][2] = np.linalg.lstsq(selected_green_points, B, rcond=1.e-10)[0]
			self.plane_params["blue"][0], self.plane_params["blue"][1], self.plane_params["blue"][2] = np.linalg.lstsq(selected_blue_points, B, rcond=1.e-10)[0]
			
		

		# Verify that each pair of 3D planes are approximately orthogonal to each other
			orthoTestRedBlue = np.dot(self.plane_params["red"], self.plane_params["blue"])
			orthoTestRedGreen = np.dot(self.plane_params["red"], self.plane_params["green"])
			orthoTestBlueGreen = np.dot(self.plane_params["blue"], self.plane_params["green"])
			
			MIN_ORTHOGONAL_THRESHOLD = 1
			
			if (orthoTestRedBlue > -MIN_ORTHOGONAL_THRESHOLD and orthoTestRedBlue < MIN_ORTHOGONAL_THRESHOLD):
				if (orthoTestRedGreen > -MIN_ORTHOGONAL_THRESHOLD and orthoTestRedBlue < MIN_ORTHOGONAL_THRESHOLD):
					if (orthoTestBlueGreen > -MIN_ORTHOGONAL_THRESHOLD and orthoTestBlueGreen < MIN_ORTHOGONAL_THRESHOLD):
						orthogonalPlanes = True
				
			if (orthogonalPlanes):
			
				# Feature detection
				# Solve 3x3 linear system of equations given by the three intersecting planes, in order to find their point of intersection
				interPoint = np.linalg.solve([self.plane_params["red"][0:3], self.plane_params["green"][0:3], self.plane_params["blue"][0:3]], np.array([1, 1, 1]))

				# Obtain z-axis (blue) vector as the vector orthogonal to the 3D plane defined by the red (x-axis) and the green (y-axis)
				### Enter your code ###
				

				# Obtain y-axis (green) vector as the vector orthogonal to the 3D plane defined by the blue (z-axis) and the red (x-axis)
				### Enter your code ###

				# Construct the 3x3 rotation matrix whose columns correspond to the x, y and z axis respectively
				### Enter your code ###

				# Obtain the corresponding euler angles from the previous 3x3 rotation matrix
				### Enter your code ###
				# use tf.transformations.euler_from_matrix

				# Set the translation part of the 6DOF pose 'self.feature_pose'
				### Enter your code ###
				#self.feature_pose.translation.x = interPoint[0]
				#self.feature_pose.translation.y = interPoint[1]
				#self.feature_pose.translation.z = interPoint[2]


				# Set the rotation part of the 6DOF pose 'self.feature_pose'
				### Enter your code ###
				# use tf.transformations.quaternion_from_euler
				self.feature_pose = Transform(Vector3(0, 0, 0.5), tf.transformations.quaternion_from_euler(0, 0, 0))

				# Publish the transform using the data stored in the 'self.feature_pose'
				self.br.sendTransform((self.feature_pose.translation.x, self.feature_pose.translation.y, self.feature_pose.translation.z), self.feature_pose.rotation, rospy.Time.now(), "corner_6dof_pose", "camera_depth_optical_frame") 

				# Empty points
				self.empty_points()

if __name__ == '__main__':
    my_estim_object = Estimation_Node('my_estimation_node')
