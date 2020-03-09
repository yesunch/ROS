#!/usr/bin/env python

from Point import Point
from PointAndNeighbours import PointAndNeighbours
import random

class Map:


	def __init__(self, grid):
		self.map = grid
		self.width = grid.info.width
		self.height = grid.info.height
		self.reso = grid.info.resolution
		
		self.points_grid = []
		
		self.origin = (grid.info.origin.position.x, grid.info.origin.position.y)
		
		
		
	def to_points_grid(self):
		# method to transfrom grid to point grid with coordonates and state
		for j in range(0, self.height):
			for i in range(0, self.width):
				self.points_grid.append(Point(i*self.reso + self.origin[0], j*self.reso + self.origin[1], i, j, self.map.data[j*self.width + i]))
				
				
	def getPointFromIndex(self, indexX, indexY):
		for p in self.points_grid:
			if (p.indexX == indexX and p.indexY == indexY):
				return p

		
		
	def getPointFromCoordonates(self, coorX, coorY):
		delta = 0.000005
		for p in self.points_grid:
			if (p.positionX - delta <= coorX <= p.positionX + delta):
				if (p.positionY - delta <= coorY <= p.positionY + delta):
					return p;
				
				
				
	def defineGoalPoint(self):
		# method to take randomly a goal point
		free_points = []
		for p in self.points_grid:
			if p.state == 0:
				free_points.append(p)
		return random.choice(free_points)
			
			
			
	def computePointNeighbours(self, p):	
		neighBours = []		
		if (p.indexX != 0 and self.getPointFromIndex(p.indexX-1, p.indexY).state == 0): 
			neighBours.append(self.getPointFromIndex(p.indexX-1, p.indexY))
		if (p.indexX != self.width-1 and self.getPointFromIndex(p.indexX+1, p.indexY).state == 0):
			neighBours.append(self.getPointFromIndex(p.indexX+1, p.indexY))
		if (p.indexY != 0 and self.getPointFromIndex(p.indexX, p.indexY-1).state == 0):
			neighBours.append(self.getPointFromIndex(p.indexX, p.indexY-1))
		if (p.indexY != self.height-1 and self.getPointFromIndex(p.indexX, p.indexY+1).state == 0):
			neighBours.append(self.getPointFromIndex(p.indexX, p.indexY+1))
		return neighBours
		
		
		
	def bfs(self, initialPoint, goalPoint):
		O = []
		C = []
		O.append(initialPoint)
		while (len(O) != 0):
			x = O.pop(0)
			C.append(x)
			if (x == goalPoint):
				return x
			else:
				for p in self.computePointNeighbours(x):
					if (p == goalPoint):
						p.parent = x
						return p
					elif (p not in O and p not in C):
						p.parent = x
						O.append(p)
		return None
		
		
		
	def recoverPathPoints(self, resultOfBfs):
		points = []
		x = resultOfBfs
		while (x.parent is not None):
			points.append(x)
			x = x.parent
		points.append(x)
		return points
		
		
		
		
	def getGridIndexFromPointIndex(self, p):
		return self.width*p.positionX + p.positionY
		
		
	def computeNewMap(self, points):
		for p in points:
			self.map[self.getGridIndexFromPointIndex(p)] = 100
		return self.grid
		
			
