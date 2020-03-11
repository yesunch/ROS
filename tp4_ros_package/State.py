#!/usr/bin/env python

class State:
	"""
	State class, to describe a state of the map.
	A state is describe with the following data :
	- its coordonates (x,y)
	- its state
	As the map topic gives us a OccupancyGrid represented as a one-dimensional array,
	we have chosen to convert it as a two-dimensional, storing in every state the indexX and indexY
	"""
	
	
	def __init__(self, posX, posY, indX, indY, state):
		"""
		Constructor of a state
		The positionX and positionY corresponds to its coordonates (x,y)
		The indexX and indexY corresponds to the position of the point in the OccupancyGrid
		The state corresponds to the current state : free, occupied or unknown
		The parent corresponds to all the parent of this state
		"""
		self.positionX = posX
		self.positionY = posY
		self.indexX = indX
		self.indexY = indY
		self.state = state
		self.parent = None
		
		
	def __hash__(self):
		"""
		Hash fonction of the State class
		"""
		return hash((self.indexX, self.indexY))
	
			
	def __eq__(self, other):
		"""
		Redefinition of the __eq__ to test the equality of two States
		"""
		return (self.positionX, self.positionY) == (other.positionX, other.positionY)
		
	def __ne__(self, other):
		"""
		Redefinition of the __ne__ to test the inequality of two States
		"""
		return not(self == other)
		
		
	def describe(self):
		"""
		Method to describe the State by printing all of its characteristics.
		"""
		return ("State of Index : (" + str(self.indexX) + "," + str(self.indexY) + "), Position " + str(self.positionX) + ", " + str(self.positionY) + " and state : " + str(self.state))
		
		
