#!/usr/bin/env python

class Point:
	
	def __init__(self, posX, posY, indX, indY, state):
		self.positionX = posX
		self.positionY = posY
		self.indexX = indX
		self.indexY = indY
		self.state = state
		
		
	def describe(self):
		return ("Point of Index : (" + str(self.indexX) + "," + str(self.indexY) + "), Position " + str(self.positionX) + ", " + str(self.positionY) + " and state : " + str(self.state))
		
		
		
	def getNeighbours(self, width, height):
		# method to get the neightbours of the point
		neighBours = []
		
		
		return neighBours
