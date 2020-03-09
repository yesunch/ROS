#!/usr/bin/env python

class State:
	
	def __init__(self, posX, posY, indX, indY, state):
		self.positionX = posX
		self.positionY = posY
		self.indexX = indX
		self.indexY = indY
		self.state = state
		self.parent = None
		
		
	def __hash__(self):
		return hash((self.indexX, self.indexY))
		
	def __eq__(self, other):
		return (self.positionX, self.positionY) == (other.positionX, other.positionY)
		
	def __ne__(self, other):
		return not(self == other)
		
		
	def describe(self):
		return ("State of Index : (" + str(self.indexX) + "," + str(self.indexY) + "), Position " + str(self.positionX) + ", " + str(self.positionY) + " and state : " + str(self.state))
		
		
