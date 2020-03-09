#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import *
from std_msgs.msg import *
from Point import Point
from Map import Map

print("Planning Process")

class Process_Node:
	            
                
	def __init__(self, node_name):
		# Constructor
		self.nname = node_name       
		self.grid_map = None
		self.isStartPointSet = False
		self.initialPose = ()
		
		self.path = []
		self.path_publisher = rospy.Publisher("new_map", OccupancyGrid, queue_size=1)
		
		rospy.init_node(node_name, anonymous=True)
		self.rate = rospy.Rate(0.5)
		#rospy.Subscriber("/odom", Odometry, self.start_point_callback)
		#self.rate.sleep()
		rospy.Subscriber("/map", OccupancyGrid, self.callback)	
		
			
		rospy.spin()
		
		"""
	def start_point_callback(self, data):
		print("START CALLBACK")
		self.initialPose = (data.pose.pose.position.x, data.pose.pose.position.y)
		self.isStartPointSet = True
		self.rate.sleep() 
		"""
		
		
	def callback(self, data):
		print("CALLBACK")
		#if (self.isStartPointSet == True):
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
		self.grid_map.to_points_grid()
		print("Size PointsMap : " + str(len(self.grid_map.points_grid)))
		print(self.grid_map.points_grid[0].describe())

		for p in self.grid_map.points_grid:
			print(p.describe())
		
		compt = 0	
		for p in self.grid_map.points_grid:
			if (p.state == 0):
				compt = compt+1
		print("Number of points with state 0 : " + str(compt))
		
		initial_point = self.grid_map.getPointFromCoordonates(self.initialPose[0], self.initialPose[1])
		print("Initial Point : " + initial_point.describe())
		goal_point = self.grid_map.defineGoalPoint()
		print("Goal Point : " + goal_point.describe())
		
		print("Launch BFS")
		points_of_path = self.grid_map.bfs(initial_point, goal_point)
		
		if (points_of_path is None):
			print("No path found")
		else:
			print("Retreive path")
			self.path = self.grid_map.recoverPathPoints(points_of_path)
		
			print("Print Path : ")
			for p in self.path:
				print(p.describe())
		
		
		self.path_publisher(self.grid_map.computeNewMap(self.path))

			
			
		

if __name__ == '__main__':
	my_process_node = Process_Node("process_node")


# size of file : 153 599
"""
STEPS TO DO :
// - read the map
// - load the map into points_grid, with Point with those parameter : coor_x, coor_y, indX, indY, state, parent
// - get the position of the robot
- get the point corresponding to that position
- get the method to return the neightbours of that point
// - define a goal point : prendre tous les points disponibles et faire random
- creathe the algo from this ==> return the point with all the parent as the way
- print those points on the map with OccupancyGrid and blacked points
"""
