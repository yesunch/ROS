#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from State import State
from Map import Map

print(">>> Planning Process Node <<< ")

class Process_Node:
	"""
	Process node class, will compute the BDS from the initial state to a randomly chosen state
	Subscribe to topic /map and publish to topic /path_bfs
	"""
	            
                
	def __init__(self, node_name):
		"""
		Constructor of the Process Node
		"""
		self.nname = node_name      		# the node name 
		self.grid_map = None				# the Map		
		self.initialPose = (0, 0)			# the initialPose of the robot
		
		self.path = []						# the path of the bfs
		
		
		rospy.init_node(node_name, anonymous=True)		# create the node
		self.path_publisher = rospy.Publisher("path_bfs", MarkerArray, queue_size=1)	# the topic we will publish the path found
		rospy.Subscriber("/map", OccupancyGrid, self.callback)		#subscribe to the map topic
		rospy.spin()												#ros loop
		
	
	
	def printPathWithMarker(self):
		"""
		Print the path with a MarkerArray and send it for visualization in the topic MarkerArray
		"""
		markers = []
		color = ColorRGBA()		# set the color
		color.r = 255
		color.g = 0
		color.b = 0
		color.a = 1
		
		marker_id = 0
		for p in self.path:
			point = Marker()	# create the marker
			# set all its parameters
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
			# put it in a Marker[]
			markers.append(point)
			marker_id = marker_id + 1
			
		markerPath = MarkerArray()	# create MarkerArray and set the markers
		markerPath.markers = markers

		return markerPath		# return the path with markers
		


	def callback(self, data):
		"""
		Callback method called when the map data is received
		"""
		print(">> Map data received")
		
		self.grid_map = Map(data)		# create the Map instance
		self.grid_map.to_States_grid()	# convert the Occupancygrid to the State Grid 
	
		initial_State = self.grid_map.getStateFromCoordonates(self.initialPose[0], self.initialPose[1])	# get the initial State from coordonates of initialPose
		if (initial_State is None):											# if the initialState is not found
			print(">> Initial state not found, choose another one")
		else:
			print(">> Initial Position defined")
			goal_State = self.grid_map.defineGoalState()					# get the goal state randomly
			print(">> Goal Position chosen randomly among free spaces") 
	
			print(">> Computing BFS...")
			States_of_path = self.grid_map.bfs(initial_State, goal_State)	# compute the BFS
	
			if (States_of_path is None):			# if not path is found
				print(">> No path found !")
			else:
				print(">> Path Found !")			# if path found
				self.path = self.grid_map.recoverPathStates(States_of_path)		# recover the path from the parents of the result of the BFS
			
				print(">> Path sent to /path_bfs topic to visualize in RViz")	
				self.path_publisher.publish(self.printPathWithMarker())		# publish the path in the /path_bfs topic


if __name__ == '__main__':
	my_process_node = Process_Node("process_node")			# create the node

