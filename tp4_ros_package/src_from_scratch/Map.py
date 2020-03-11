#!/usr/bin/env python

from State import State
import random

class Map:
	"""
	The Map class represents the map around the robot.
	It will store the OccupancyGrid and its property, and will convert to the State_grid, where every point of the grid is a State object
	"""

	def __init__(self, grid):
		self.map = grid					# the occupancy grid
		self.width = grid.info.width	# the width of the grid
		self.height = grid.info.height	# the height of the grid
		self.reso = grid.info.resolution	# the resolution of the grid
		
		self.States_grid = []			# list to store the state_grid, meaning the occupancy grid where each point is a State object
		
		self.origin = (grid.info.origin.position.x, grid.info.origin.position.y)		# the origin of the map
		
		
		
	def to_States_grid(self):
		"""
		Transform the occupancy grid to the state_grid, creating a State instance for every point according to its position in the occupancy grid
		"""
		for j in range(0, self.height):		# we iterate on every line
			for i in range(0, self.width):	# we iterate on every column
				self.States_grid.append(State(i*self.reso + self.origin[0], j*self.reso + self.origin[1], i, j, self.map.data[j*self.width + i]))
				
				
	def getStateFromIndex(self, indexX, indexY):
		"""
		Get a state from the state_grid using its index X and Y
		"""
		for s in self.States_grid:
			if (s.indexX == indexX and s.indexY == indexY):
				return s

		
		
	def getStateFromCoordonates(self, coorX, coorY):
		"""
		Get a state from the state_grid using points coordonates
		"""
		delta = 0.000005		# we use a little delta to get the point because it may not be exactly equals to the coorX and coorY
		for s in self.States_grid:
			if (s.positionX - delta <= coorX <= s.positionX + delta):
				if (s.positionY - delta <= coorY <= s.positionY + delta):
					return s;
				
				
				
	def defineGoalState(self):
		"""
		Choose randomly a goal state from all the free state of the map
		"""
		free_States = []
		for s in self.States_grid:
			if s.state == 0:			# we get the free state with state == 0
				free_States.append(s)
		return random.choice(free_States)	# get randomly a free state
			
			
			
	def computeStateNeighbours(self, s):	
		"""
		Get the free neighBours for a given State s : will only give the state were the robot can move
		For every movement (right, up, left, down), we check that the state exits (not in a corner) and that it is free
		"""
		neighBours = []		
		if (s.indexX != 0 and self.getStateFromIndex(s.indexX-1, s.indexY).state == 0): 	
			neighBours.append(self.getStateFromIndex(s.indexX-1, s.indexY))
		if (s.indexX != self.width-1 and self.getStateFromIndex(s.indexX+1, s.indexY).state == 0):
			neighBours.append(self.getStateFromIndex(s.indexX+1, s.indexY))
		if (s.indexY != 0 and self.getStateFromIndex(s.indexX, s.indexY-1).state == 0):
			neighBours.append(self.getStateFromIndex(s.indexX, s.indexY-1))
		if (s.indexY != self.height-1 and self.getStateFromIndex(s.indexX, s.indexY+1).state == 0):
			neighBours.append(self.getStateFromIndex(s.indexX, s.indexY+1))
		return neighBours
		
		
		
	def bfs(self, initialState, goalState):
		"""
		The BFS algorithm to perform the search of the sorthest path
		We use the description of the BDS given in the course
		"""
		O = []
		C = []
		O.append(initialState)
		while (len(O) != 0):		# check if O is empty
			x = O.pop(0)
			C.append(x)
			if (x == goalState):
				return x
			else:
				for p in self.computeStateNeighbours(x):	# get every move for the current state
					if (p == goalState):
						p.parent = x						# update the parent
						return p
					elif (p not in O and p not in C):
						p.parent = x						# update the parent
						O.append(p)
		return None
		
		
		
	def recoverPathStates(self, resultOfBfs):
		"""
		After computation of the BDS, we need to recover the path from the resulting point
		For the result point x of BDS, we recover all the parent point until it comes back to the inital State
		"""
		States = []
		x = resultOfBfs
		while (x.parent is not None):
			States.append(x)
			x = x.parent
		States.append(x)
		return States
			
