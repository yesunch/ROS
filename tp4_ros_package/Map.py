#!/usr/bin/env python

from State import State
import random

class Map:
	"""
	The Map class represents the map around the robot.
	"""

	def __init__(self, grid):
		self.map = grid
		self.width = grid.info.width
		self.height = grid.info.height
		self.reso = grid.info.resolution
		
		self.States_grid = []
		
		self.origin = (grid.info.origin.position.x, grid.info.origin.position.y)
		
		
		
	def to_States_grid(self):
		# method to transfrom grid to State grid with coordonates and state
		for j in range(0, self.height):
			for i in range(0, self.width):
				self.States_grid.append(State(i*self.reso + self.origin[0], j*self.reso + self.origin[1], i, j, self.map.data[j*self.width + i]))
				
				
	def getStateFromIndex(self, indexX, indexY):
		for p in self.States_grid:
			if (p.indexX == indexX and p.indexY == indexY):
				return p

		
		
	def getStateFromCoordonates(self, coorX, coorY):
		delta = 0.000005
		for p in self.States_grid:
			if (p.positionX - delta <= coorX <= p.positionX + delta):
				if (p.positionY - delta <= coorY <= p.positionY + delta):
					return p;
				
				
				
	def defineGoalState(self):
		# method to take randomly a goal State
		free_States = []
		for p in self.States_grid:
			if p.state == 0:
				free_States.append(p)
		return random.choice(free_States)
			
			
			
	def computeStateNeighbours(self, p):	
		neighBours = []		
		if (p.indexX != 0 and self.getStateFromIndex(p.indexX-1, p.indexY).state == 0): 
			neighBours.append(self.getStateFromIndex(p.indexX-1, p.indexY))
		if (p.indexX != self.width-1 and self.getStateFromIndex(p.indexX+1, p.indexY).state == 0):
			neighBours.append(self.getStateFromIndex(p.indexX+1, p.indexY))
		if (p.indexY != 0 and self.getStateFromIndex(p.indexX, p.indexY-1).state == 0):
			neighBours.append(self.getStateFromIndex(p.indexX, p.indexY-1))
		if (p.indexY != self.height-1 and self.getStateFromIndex(p.indexX, p.indexY+1).state == 0):
			neighBours.append(self.getStateFromIndex(p.indexX, p.indexY+1))
		return neighBours
		
		
		
	def bfs(self, initialState, goalState):
		O = []
		C = []
		O.append(initialState)
		while (len(O) != 0):
			x = O.pop(0)
			C.append(x)
			if (x == goalState):
				return x
			else:
				for p in self.computeStateNeighbours(x):
					if (p == goalState):
						p.parent = x
						return p
					elif (p not in O and p not in C):
						p.parent = x
						O.append(p)
		return None
		
		
		
	def recoverPathStates(self, resultOfBfs):
		States = []
		x = resultOfBfs
		while (x.parent is not None):
			States.append(x)
			x = x.parent
		States.append(x)
		return States
		
		
		
		
	def getGridIndexFromStateIndex(self, p):
		return self.width*p.positionX + p.positionY
		
		
	def computeNewMap(self, States):
		for p in States:
			self.map[self.getGridIndexFromStateIndex(p)] = 100
		return self.grid
		
			
