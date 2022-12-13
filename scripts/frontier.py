import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import plot_line_segments

class frontier(object):
	def __init__(self, x_init, occupancy)->None:
		self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
		self.robot_start_x = int((x_init[0] - self.occupancy.origin_x) / self.occupancy.resolution)    # initial state
		self.robot_start_y = int((x_init[1] - self.occupancy.origin_y) / self.occupancy.resolution)
		self.random_jump = 0.1
	
	def is_frontier(self, x, y):
		"""
		Checks if a give grid position (x, y) is a frontier, meaning it is inside the bounds of the map and
		is on the boundary between known-empty and unknown.
		Inputs:
			x: x grid position
			y: y grid position
		Output:
			Boolean True/False
		Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
			  useful here
		"""
		########## Code starts here ##########
		if x < 0:
			return False
		if x >= self.occupancy.width:
			return False
		if y < 0:
			return False
		if y >= self.occupancy.height:
			return False

		if (self.occupancy.is_free_grid(x,y) and not self.occupancy.is_unknown_grid(x,y,1)):
			for dx in range(-1,2):
				for dy in range(-1,2):
					if (self.occupancy.is_unknown_grid(x + dx, y + dy, 1)):
						return True

		return False

	def get_frontier_centroid(self):
		count = 0
		centroid_x = 0
		centroid_y = 0
		for x in range(0, self.occupancy.width):
			for y in range(0, self.occupancy.height):
				if self.is_frontier(x,y):
					force = 1/((self.robot_start_x - x)**2 + (self.robot_start_y - y)**2)
					centroid_x += x * force
					centroid_y += y * force
					count += force
		
		centroid = (0,0)
		if count > 0:
			centroid_x = centroid_x/count
			centroid_y = centroid_y/count
			x_c = centroid_x * self.occupancy.resolution + self.occupancy.origin_x
			y_c = centroid_y * self.occupancy.resolution + self.occupancy.origin_y
			centroid = (x_c, y_c)
		else:
			print("Frontier Count = 0")

		print("Robot grid points: (", self.robot_start_x, self.robot_start_y, ")")
		print("Centroid frontier: (", centroid_x, centroid_y, ")")

		while (not self.occupancy.is_free(centroid)):
			print("centroid in the wall, jumping")
			centroid_jump_x = centroid[0] + np.random.uniform(-self.random_jump,self.random_jump)
			centroid_jump_y = centroid[1] + np.random.uniform(-self.random_jump,self.random_jump)
			centroid = (centroid_jump_x,centroid_jump_y)

		return (count > 0, centroid)

	def get_frontier_closest(self):
		closest_r = 1e16
		closest_x = 0
		closest_y = 0
		count = 0
		for x in range(0, self.occupancy.width):
			for y in range(0, self.occupancy.height):
				if self.is_frontier(x,y):
					r = (self.robot_start_x - x)**2 + (self.robot_start_y - y)**2
					if (r < closest_r):
						closest_x = x
						closest_y = y
						closest_r = r
						count += 1
		print("Robot grid points: (", self.robot_start_x, self.robot_start_y, ")")
		print("Robot closest frontier: (", closest_x, closest_y, ")")
		self.occupancy.print_near(closest_x,closest_y, 5)
		
		x_c = closest_x * self.occupancy.resolution + self.occupancy.origin_x
		y_c = closest_y * self.occupancy.resolution + self.occupancy.origin_y
		closest = (x_c, y_c)
		while (not self.occupancy.is_free(closest)):
			print("closest in the wall, jumping")
			closest_jump_x = closest[0] + np.random.uniform(-self.random_jump,self.random_jump)
			closest_jump_y = closest[1] + np.random.uniform(-self.random_jump,self.random_jump)
			closest = (closest_jump_x,closest_jump_y)
		return (count > 0, closest)
