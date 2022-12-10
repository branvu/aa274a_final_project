import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import plot_line_segments

class frontier(object):
	def __init__(self, x_init, occupancy)->None:
		self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
		self.x_init = self.snap_to_grid(x_init)    # initial state
	
	def is_frontier(self, x):
		"""
		Checks if a give state x is a frotntier, meaning it is inside the bounds of the map and
		is on the boundary between known-empty and unknown.
		Inputs:
			x: state tuple
		Output:
			Boolean True/False
		Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
			  useful here
		"""
		########## Code starts here ##########
		for dim in range(len(x)):
			if x[dim] < self.statespace_lo[dim]:
				return False
			if x[dim] > self.statespace_hi[dim]:
				return False

		if (self.occupancy.is_free(x)):
			for dx in range(-1,2):
				for dy in range(-1,2):
					neighbor = self.snap_to_grid((x[0] + self.resolution * dx, x[1] + self.resolution * dy))
					if (self.occupancy.is_unknown(neighbor)):
						return True

		return False

	def snap_to_grid(self, x):
		""" Returns the closest point on a discrete state grid
		Input:
			x: tuple state
		Output:
			A tuple that represents the closest point to x on the discrete state grid
		"""
		snapped = (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))
		#print("snapped:", snapped)
		return snapped

	def get_frontier_centroid(self):
		count = 0
		centroid_x = 0
		centroid_y = 0
		for x in range(self.statespace_lo[0], self.statespace_hi[0]):
			for y in range(self.statespace_lo[1], self.statespace_hi[1]):
				if self.is_frontier((x,y)):
					centroid_x += x
					centroid_y += y
					count += 1
		centroid = (0,0)
		if count > 0:
			centroid = (centroid_x/count, centroid_y/count)
		return (count > 0, centroid) 
