#!/usr/bin/env python

from Point import Point

class Map:


	def __init__(self, grid):
		self.map = grid
		self.width = grid.info.width
		self.height = grid.info.height
		self.reso = grid.info.resolution
		
		self.points_grid = []
		
		self.origin = (grid.info.origin.position.x, grid.info.origin.position.y)
		
		#self.origin = Point(grid.info.origin.position.x, grid.info.origin.position.y, grid.data[0])
		#self.origin.describe()
		
	def to_points_grid(self):
		# method to transfrom grid to point grid with coordonates and state
		for j in range(0, self.height):
			for i in range(0, self.width):
				self.points_grid.append(Point(i*self.reso + self.origin[0], j*self.reso + self.origin[1], i, j, self.map.data[j*self.width + i]))
				
	def getPointFromIndex(self, indexX, indexY):
		for p in self.points_grid:
			if (p.indexX == indexX and p.indexY == indexY):
				return p
		return Point();
				
			
	def computePointNeighbours(self, p):	
		neighBours = []		
		if (p.indexX != 0): 
			neighBours.append(self.getPointFromIndex(p.indexX-1, p.indexY))
		if (p.indexX != self.width-1):
			neighBours.append(self.getPointFromIndex(p.indexX+1, p.indexY))
		if (p.indexY != 0):
			neighBours.append(self.getPointFromIndex(p.indexX, p.indexY-1))
		if (p.indexY != self.height-1):
			neighBours.append(self.getPointFromIndex(p.indexX, p.indexY+1))
		return neighBours
			
			
			
			
			
			
			
			
			
			

		
