from rrt import build_obstacle_course
from rrt import add_start_and_goal
import matplotlib.pyplot as plt
from matplotlib.path import Path
import sys, random, math

class RRT():
	def __init__(self):
		fig, ax = plt.subplots()
		build_obstacle_course( './world_obstacles.txt', ax)
		start, goal = add_start_and_goal('./start_goal.txt', ax)

		obstacles = self.readObstacles('./world_obstacles.txt')
		self.get_rrt(start, goal, 0.1, obstacles)


	def readObstacles(self, fileName):
		obstacles = []
		file = open(fileName, 'r')
		numObstacles = file.readline()
		i = 0
		end = 0
		first = (0,0)
		for line in file:
			l = line[:-2].split(" ")
			if len(l) == 2:
				i += 1
				obstacles.append((int(l[0]), int(l[1])))
				if i == 1: 
					first = (int(l[0]), int(l[1]))
				if i == end:
					obstacles.append(first)
			if len(l) == 1:
				i = 0
				end = int(l[0])
				obstacles.append(None)
		return obstacles

	def get_nearest_neighbor(self, path, point):
		min_distance = sys.maxsize
		min_index = -1
		for index in range(len(path)):
			cur = path[index][0];
			distance = math.sqrt((cur[0] - point[0])**2 + (cur[1] - point[1])**2)
			if distance < min_distance:
				min_distance = distance
				min_index = index
		return path[min_index][0]

	def get_expanded_point(self, nearest_point, point, distance):
		while point[0] == nearest_point[0] and point[1] == nearest_point[1]:
			point = [random.randint(0, 601), random.randint(0, 601)]

		if (point[0] == nearest_point[0]):
			if (point[1] - nearest_point[1] > 0):
				return [nearest_point[0], nearest_point[1] + distance]
			else:
				return [nearest_point[0], nearest_point[1] - distance]
		else:
			m = (point[1] - nearest_point[1]) / (point[0] - nearest_point[0])
			r = math.sqrt(1 + m**2)
			x = nearest_point[0] + distance / r
			y = nearest_point[1] + (distance * m) / r
			return [x, y]

	def reach_goal(self, point, goal, distance):
		d = math.sqrt((goal[0] - point[0])**2 + (goal[1] - point[1])**2)
		if (d <= distance):
			return True
		else:
			return False

	def get_rrt(self, start, goal, distance, obstacles):
		path = []
		path.append([start, start])
		expanded_point = start
		while not self.reach_goal(expanded_point, goal, distance):
			random_point = [random.randint(0, 601), random.randint(0, 601)]
			nearest_point = self.get_nearest_neighbor(path, random_point)
			expanded_point = self.get_expanded_point(nearest_point, random_point, distance)
			print(nearest_point, expanded_point)
			if (self.pathPlanner(nearest_point, expanded_point, obstacles)):
				path.append([nearest_point, expanded_point])
		path.append([expanded_point, goal])
		print(path)

	
	"""Path Planner"""
		# check if b lies on segment a, c
	# given b is colinear to a, c
	def onSegment(self, a, b, c):
		if (b[0] <= max(a[0], c[0]) and b[0] >= min(a[0], c[0]) and
			b[1] <= max(a[1], c[1]) and b[1] >= min(a[1], c[1])):
			return True
		return False

	# return 0 if colinear
	# return 1 if clockwise
	# return 2 if counterclockwise
	def orient(self, a, b, c):
		# check slope
		val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
		if val == 0:
			return 0
		elif val > 0:
			return 1
		else:
			return 2

	# check intersection of a1b1 and a2b2
	def intersection(self, a1, b1, a2, b2):
		# if (a1 == a2 or a1 == b2 or b1 == a2 or b1 == b2):
		# 	return False
		o1 = self.orient(a1, b1, a2)
		o2 = self.orient(a1, b1, b2)
		o3 = self.orient(a2, b2, a1)
		o4 = self.orient(a2, b2, b1)
		if (o1 != o2 and o3 != o4):
			return True
		if (o1 == 0 and self.onSegment(a1, a2, b1)):
			return True
		if (o2 == 0 and self.onSegment(a1, b2, b1)):
			return True
		if (o3 == 0 and self.onSegment(a2, a1, b2)):
			return True
		if (o4 == 0 and self.onSegment(a2, b1, b2)):
			return True
		return False

	# check boundary
	def checkBoundary(self, point1, point2):
		if 0 <= point1[0] <= 600 and 0 <= point1[1] <= 600 and 0 <= point2[0] <= 600 and 0 <= point2[0] <= 600:
			return True
		return False

	# local path planner
	# return True when legal move
	def pathPlanner(self, obstacle_points, point1, point2):
		if not self.checkBoundary(point1, point2):
			return False
		legal_move = True
		for i in range(len(obstacle_points) - 1):
			if obstacle_points[i] == None or obstacle_points[i+1] == None:
				continue
			if self.intersection(obstacle_points[i], obstacle_points[i+1], point1, point2):
				legal_move = False
				break
		return legal_move




if __name__ == '__main__':
	RRT()