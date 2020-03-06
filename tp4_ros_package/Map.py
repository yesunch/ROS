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
		for p in self.points_grid:
			if (p.positionX == coorX and p.positionY == coorY):
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
			
