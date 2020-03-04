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
		rospy.init_node(node_name, anonymous=True)
		rospy.Subscriber("/map", OccupancyGrid, self.callback)
		rospy.spin()
		
		
	def callback(self, data):
		print("Data received !")
		print("Width : " + str(data.info.width))
		print("Height : " + str(data.info.height))
		print("Resolution : " + str(data.info.resolution))
		print("Origin X : " + str(data.info.origin.position.x))
		print("Origin Y : " + str(data.info.origin.position.y))
		print("Size : " + str(len(data.data)))
		#print(data.data)
		print("Beginning of the data")
		for i in range(0, 10):
			print(data.data[i])
		print(data.data[data.info.width*10 + 25])
		# here we try to transfrom the map to a graph
		grid_map = Map(data)
		grid_map.to_points_grid()
		print("Size PointsMap : " + str(len(grid_map.points_grid)))
		print("Beginning of the map :")
		for i in range(0, 10):
			print(grid_map.points_grid[i].describe())
		print(grid_map.points_grid[data.info.width*10 + 25].describe())
		print(grid_map.getPointFromIndex(25, 10).describe())
		
		neightBoursOfA = grid_map.computePointNeighbours(grid_map.getPointFromIndex(25, 10))
		for i in (neightBoursOfA):
			print(i.describe())
		

if __name__ == '__main__':
	my_process_node = Process_Node("process_node")


# size of file : 153 599
"""
STEPS TO DO :
- create a class architecture
- method to translate from 1D to 2D
- method to translate from 2D to 1D
- get the current position
- get the desired position

- perform the BFS
- create the new map (with path points in black)
- send the map to new topic


"""
