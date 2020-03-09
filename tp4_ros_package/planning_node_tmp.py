#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from State import State
from Map import Map

print("Planning Process")

class Process_Node:
	            
                
	def __init__(self, node_name):
		# Constructor
		self.nname = node_name       
		self.grid_map = None
		self.isStartStateSet = False
		self.initialPose = ()
		
		self.path = []
		self.path_publisher = rospy.Publisher("path_bfs", MarkerArray, queue_size=1)
		
		rospy.init_node(node_name, anonymous=True)
		self.rate = rospy.Rate(0.5)
		#rospy.Subscriber("/odom", Odometry, self.start_State_callback)
		#self.rate.sleep()
		rospy.Subscriber("/map", OccupancyGrid, self.callback)	
		
			
		rospy.spin()
		
		"""
	def start_State_callback(self, data):
		print("START CALLBACK")
		self.initialPose = (data.pose.pose.position.x, data.pose.pose.position.y)
		self.isStartStateSet = True
		self.rate.sleep() 
		"""
		
		
		
		
		
	def printPathWithMarker(self):
		markers = []
		color = ColorRGBA()
		color.r = 255
		color.g = 0
		color.b = 0
		color.a = 1
		
		marker_id = 0
		for p in self.path:
			point = Marker()
			
			point.pose.position.x = p.positionX
			point.pose.position.y = p.positionY
			point.pose.position.z = 0.0
			point.pose.orientation.x = 0.0
			point.pose.orientation.y = 0.0
			point.pose.orientation.z = 0.0
			point.pose.orientation.w = 1.0
			point.scale.x = 0.1
			point.scale.y = 0.1
			point.scale.z = 0.1
			point.color = color
			point.header.frame_id = "map"
			point.header.stamp = rospy.Time.now()
			point.type = Marker.CUBE
			point.action = point.ADD
			point.id = marker_id
			
			markers.append(point)
			marker_id = marker_id + 1
			
		markerPath = MarkerArray()
		markerPath.markers = markers

		return markerPath
		
		
		
		
		
	def callback(self, data):
		print("CALLBACK")
		#if (self.isStartStateSet == True):
		print("Data received !")
		print("Width : " + str(data.info.width))
		print("Height : " + str(data.info.height))
		print("Resolution : " + str(data.info.resolution))
		print("Origin X : " + str(data.info.origin.position.x))
		print("Origin Y : " + str(data.info.origin.position.y))
		print("Size : " + str(len(data.data)))
		#print(data.data)
		#self.initialPose = (data.info.origin.position.x, data.info.origin.position.y)
		self.initialPose = (0, 0)

		# here we try to transfrom the map to a graph
		self.grid_map = Map(data)
		self.grid_map.to_States_grid()
		print("Size StatesMap : " + str(len(self.grid_map.States_grid)))
		print(self.grid_map.States_grid[0].describe())

		for p in self.grid_map.States_grid:
			print(p.describe())
		
		compt = 0	
		for p in self.grid_map.States_grid:
			if (p.state == 0):
				compt = compt+1
		print("Number of States with state 0 : " + str(compt))
		
		initial_State = self.grid_map.getStateFromCoordonates(self.initialPose[0], self.initialPose[1])
		print("Initial State : " + initial_State.describe())
		goal_State = self.grid_map.defineGoalState()
		print("Goal State : " + goal_State.describe())
		
		print("Launch BFS")
		States_of_path = self.grid_map.bfs(initial_State, goal_State)
		
		if (States_of_path is None):
			print("No path found")
		else:
			print("Retreive path")
			self.path = self.grid_map.recoverPathStates(States_of_path)
		
			print("Print Path : ")
			for p in self.path:
				print(p.describe())
		
		
		self.path_publisher.publish(self.printPathWithMarker())

	
	
	
	
			
		

if __name__ == '__main__':
	my_process_node = Process_Node("process_node")


# size of file : 153 599
"""
STEPS TO DO :
// - read the map
// - load the map into States_grid, with State with those parameter : coor_x, coor_y, indX, indY, state, parent
// - get the position of the robot
- get the State corresponding to that position
- get the method to return the neightbours of that State
// - define a goal State : prendre tous les States disponibles et faire random
- creathe the algo from this ==> return the State with all the parent as the way
- print those States on the map with OccupancyGrid and blacked States
"""
