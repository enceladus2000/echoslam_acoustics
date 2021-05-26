import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

def add_obstacle(box, obstacle_faces, material):
	for face in obstacle_faces:
		box.walls.append(pra.wall_factory(face.T, 
					material.energy_absorption["coeffs"], 
					material.scattering["coeffs"]))

def make_polygon(centre, height, radius, N=3, theta=0):
	lower_points = []
	upper_points = []
	
	for n in range(N):
		x = radius * np.cos(2*np.pi*n/N + theta) + centre[0]
		y = radius * np.sin(2*np.pi*n/N + theta) + centre[1]

		lower_points.append(np.array([x, y, centre[2]]))
		upper_points.append(np.array([x, y, centre[2] + height]))

	walls = []
	# add side walls
	for i in range(N-1):
		walls.append(np.array([
					lower_points[i], upper_points[i],
					upper_points[i+1], lower_points[i+1]
				]))
	# last side wall
	walls.append(np.array([
					lower_points[N-1], upper_points[N-1],
					upper_points[0],lower_points[0] 
				]))
	# lower and upper walls
	walls.append(np.array(lower_points))
	walls.append(np.array(upper_points))

	return walls