import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import plot_line_segments

class frontier(object):
	def __init__(self, x_init, occupancy)->None:
		self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
		print("Occupancy Sum Front:", np.sum(self.occupancy.probs))
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

		if (self.occupancy.is_free_grid(x,y,1) and not self.occupancy.is_unknown_grid(x,y,1)):
			for dx in range(-1,2):
				for dy in range(-1,2):
					if (self.occupancy.is_unknown_grid(x + dx, y + dy, 1)):
						return True

		return False

	def get_frontier_closest(self):
		start_time = time.time()
		valid_fronts = self.occupancy.find_all_frontiers()
		count = valid_fronts.shape[0]
		#print("Valid Fronts", valid_fronts)
		#print("Frontier Find Time", time.time() - start_time)

		if (count <= 0):
			return (False, None)

		distances = (valid_fronts - np.array([self.robot_start_x, self.robot_start_y]))
		distances = np.linalg.norm(distances, axis=1)
		close_point = valid_fronts[np.argmin(distances)]
		closest_x = close_point[0]
		closest_y = close_point[1]

		print("Robot grid points: (", self.robot_start_x, self.robot_start_y, ")")
		print("Robot closest frontier: (", closest_x, closest_y, ")")
		#self.occupancy.print_near(closest_x,closest_y, 7)
		#self.occupancy.convolve_near(closest_x,closest_y, 7)
		
		#closest_x, closest_y = self.occupancy.find_nearest_free(closest_x, closest_y)
		#print("Nearest free frontier: (", closest_x, closest_y, ")")
		#self.occupancy.print_near(closest_x,closest_y, 7)
		
		x_c = closest_x * self.occupancy.resolution + self.occupancy.origin_x
		y_c = closest_y * self.occupancy.resolution + self.occupancy.origin_y
		closest = (x_c, y_c)
		#print("Time to find", time.time() - start_time)
		#print("Origin, Resolution:", self.occupancy.origin_x, self.occupancy.origin_y, self.occupancy.resolution)
		print("Returning goal: (", x_c, y_c, ")")
		return (True, closest)


	'''def get_frontier_centroid(self):
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
		else:
			print("Frontier Count = 0")

		print("Robot grid points: (", self.robot_start_x, self.robot_start_y, ")")
		print("Centroid frontier: (", centroid_x, centroid_y, ")")

		centroid_x, centroid_y = self.occupancy.find_nearest_free(centroid_x, centroid_y)
		print("Nearest free frontier: (", centroid_x, centroid_y, ")")

		x_c = centroid_x * self.occupancy.resolution + self.occupancy.origin_x
		y_c = centroid_y * self.occupancy.resolution + self.occupancy.origin_y
		centroid = (x_c, y_c)

		return (count > 0, centroid)

	def get_frontier_closest(self):
		start_time = time.time()
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
		closest_x = 
		closest_y = y
		print("Scan Time", time.time() - start_time)
		print("MapSize", self.occupancy.width,self.occupancy.height)
		print("Robot grid points: (", self.robot_start_x, self.robot_start_y, ")")
		print("Robot closest frontier: (", closest_x, closest_y, ")")
		self.occupancy.print_near(closest_x,closest_y, 5)
		
		closest_x, closest_y = self.occupancy.find_nearest_free(closest_x, closest_y)
		print("Nearest free frontier: (", closest_x, closest_y, ")")
		
		x_c = closest_x * self.occupancy.resolution + self.occupancy.origin_x
		y_c = closest_y * self.occupancy.resolution + self.occupancy.origin_y
		closest = (x_c, y_c)
		print("Time to find", time.time() - start_time)
		return (count > 0, closest)'''
