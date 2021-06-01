import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

def add_obstacle(box, obstacle_faces, material):
	for face in obstacle_faces:
		box.walls.append(pra.wall_factory(face.T, 
					material.energy_absorption["coeffs"], 
					material.scattering["coeffs"]))

def make_polygon(centre, radius, height, N=3, rpy=[0,0,0]):
	"""TODO: nice docstring
	"""
	lower_points = []
	upper_points = []
	
	for n in range(N):
		x = radius * np.cos(2*np.pi*n/N)
		y = radius * np.sin(2*np.pi*n/N)

		lower_points.append(np.array([x, y, height/2]))
		upper_points.append(np.array([x, y, -height/2]))

	# do rotation and translation
	cr, cp, cy = np.cos(rpy)
	sr, sp, sy = np.sin(rpy)
	Rx = np.array([
		[1, 0, 0],
		[0, cr, -sr],
		[0, sr, cr]
	]).T
	Ry = np.array([
		[cp, 0, sp],
		[0, 1, 0],
		[-sp, 0, cp]
	]).T
	Rz = np.array([
		[cy, -sy, 0],
		[sy, cy, 0],
		[0, 0, 1]
	]).T
	lower_points = np.array(lower_points) @ Rx @ Ry @ Rz + np.array(centre)
	upper_points = np.array(upper_points) @ Rx @ Ry @ Rz + np.array(centre)

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

def make_cylinder(centre, radius, height, rpy=[0,0,0], N=100):
	return make_polygon(centre, radius, height, N=N, rpy=rpy) 