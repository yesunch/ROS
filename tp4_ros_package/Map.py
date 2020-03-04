#!/usr/bin/env python

from Point import Point

class Map:


	def __init__(self, grid):
		self.map = grid
		self.width = grid.info.width
		self.height = grid.info.height
		self.reso = grid.info.resolution
		
		self.origin = (grid.info.origin.position.x, grid.info.origin.position.y)
		
		#self.origin = Point(grid.info.origin.position.x, grid.info.origin.position.y, grid.data[0])
		#self.origin.describe()
		
	def to_points_grid(self):
		# method to transfrom grid to point grid with coordonates and state
		points_grid = []
		for j in range(0, self.height):
			for i in range(0, self.width):
				points_grid.append(Point(i*self.reso + self.origin[0], j*self.reso + self.origin[1], self.map.data[i*self.width + j]))
		return points_grid
