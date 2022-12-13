import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        for obs in self.obstacles:
            ax = fig.add_subplot(111, aspect='equal')
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))

class StochOccupancyGrid2D(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.5):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.probs = np.reshape(np.asarray(probs), (height, width))
        self.window_size = window_size
        self.thresh = thresh
        

    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def is_free(self, state):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        x, y = self.snap_to_grid(state)
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)

        half_size = int(round((self.window_size-1)/2))
        grid_x_lower = max(0, grid_x - half_size)
        grid_y_lower = max(0, grid_y - half_size)
        grid_x_upper = min(self.width, grid_x + half_size + 1)
        grid_y_upper = min(self.height, grid_y + half_size + 1)

        prob_window = self.probs[grid_y_lower:grid_y_upper, grid_x_lower:grid_x_upper]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total) < self.thresh

    def is_free_grid(self, grid_x, grid_y, window_size = -1):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        if window_size == -1:
            window_size = self.window_size
        grid_x = int(grid_x)
        grid_y = int(grid_y)
        half_size = int(round((window_size-1)/2))
        grid_x_lower = max(0, grid_x - half_size)
        grid_y_lower = max(0, grid_y - half_size)
        grid_x_upper = min(self.width, grid_x + half_size + 1)
        grid_y_upper = min(self.height, grid_y + half_size + 1)

        prob_window = self.probs[grid_y_lower:grid_y_upper, grid_x_lower:grid_x_upper]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total) < self.thresh

    def is_free_grid_val(self, grid_x, grid_y, window_size = -1):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        if window_size == -1:
            window_size = self.window_size
        grid_x = int(grid_x)
        grid_y = int(grid_y)
        half_size = int(round((window_size-1)/2))
        grid_x_lower = max(0, grid_x - half_size)
        grid_y_lower = max(0, grid_y - half_size)
        grid_x_upper = min(self.width, grid_x + half_size + 1)
        grid_y_upper = min(self.height, grid_y + half_size + 1)

        prob_window = self.probs[grid_y_lower:grid_y_upper, grid_x_lower:grid_x_upper]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total)

    def is_unknown_grid(self, grid_x, grid_y, window_size = -1):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        if window_size == -1:
            window_size = self.window_size
        grid_x = int(grid_x)
        grid_y = int(grid_y)
        half_size = int(round((window_size-1)/2))
        grid_x_lower = max(0, grid_x - half_size)
        grid_y_lower = max(0, grid_y - half_size)
        grid_x_upper = min(self.width, grid_x + half_size + 1)
        grid_y_upper = min(self.height, grid_y + half_size + 1)

        prob_window = self.probs[grid_y_lower:grid_y_upper, grid_x_lower:grid_x_upper]
        p_avg = np.average(np.average(np.less(prob_window,0)))
        return p_avg >= 0.5

    def print_near(self, grid_x, grid_y, window_size = -1):
        if window_size == -1:
            window_size = self.window_size
        grid_x = int(grid_x)
        grid_y = int(grid_y)
        half_size = int(round((window_size-1)/2))
        grid_x_lower = max(0, grid_x - half_size)
        grid_y_lower = max(0, grid_y - half_size)
        grid_x_upper = min(self.width, grid_x + half_size + 1)
        grid_y_upper = min(self.height, grid_y + half_size + 1)

        prob_window = self.probs[grid_y_lower:grid_y_upper, grid_x_lower:grid_x_upper]
        
        print(prob_window)

    def find_nearest_free(self, grid_x, grid_y):
        while(not self.is_free_grid(grid_x, grid_y)):
            min_val = 1e16
            min_dx = 0
            min_dy = 0
            for dx in range(-1,2):
				for dy in range(-1,2):
                    new_val = self.is_free_grid_val(grid_x + dx, grid_y + dy)
					if (new_val < min_val):
                        min_dx = dx
                        min_dy = dy
                        min_val = new_val
            grid_x += min_dx
            grid_y += min_dx
        return (grid_x, grid_y)

                        

    def is_(self, state):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        x, y = self.snap_to_grid(state)
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)

        half_size = int(round((self.window_size-1)/2))
        grid_x_lower = max(0, grid_x - half_size)
        grid_y_lower = max(0, grid_y - half_size)
        grid_x_upper = min(self.width, grid_x + half_size + 1)
        grid_y_upper = min(self.height, grid_y + half_size + 1)

        prob_window = self.probs[grid_y_lower:grid_y_upper, grid_x_lower:grid_x_upper]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total) < self.thresh

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        pts = []
        for i in range(self.probs.shape[0]):
            for j in range(self.probs.shape[1]):
                # convert i to (x,y)
                x = j * self.resolution + self.origin_x
                y = i * self.resolution + self.origin_y
                if not self.is_free((x,y)):
                    pts.append((x,y))
        pts_array = np.array(pts)
        plt.scatter(pts_array[:,0],pts_array[:,1],color="red",zorder=15,label='planning resolution')
        plt.xlim([self.origin_x, self.width * self.resolution + self.origin_x])
        plt.ylim([self.origin_y, self.height * self.resolution + self.origin_y])
