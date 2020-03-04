#!/usr/bin/env python

class Point:
	
	def __init__(self, posX, posY, state):
		self.positionX = posX
		self.positionY = posY
		self.state = state
		
		
	def describe(self):
		return ("Point of Position " + str(self.positionX) + ", " + str(self.positionY) + " and state : " + str(self.state))
