import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

def create_walls(obstacle_faces, material):
	"""Returns a list of Wall objects that can be used in the Room constructor

	Args:
		obstacle_faces (array of Nx3 numpy 2D matrices): Same as the output of make_polygon. 
			Each element of this is a 2D matrix representing a wall, each row is a coordinate
			of a vertex.
		material (pra.Material): Material of the wall

	Returns:
		list of Wall objects: Can be directly use in Room constructor
	"""
	walls = []
	for face in obstacle_faces:
		walls.append(
			pra.wall_factory(
				face.T, 
				material.energy_absorption["coeffs"], 
				material.scattering["coeffs"]
			)
		)

	return walls 

def make_polygon(centre, radius, height, N=3, rpy=[0,0,0], reverse_normals=False):
	"""Create an extruded polygon

	Args:
		centre (array-like): centre of mass of polygon
		radius (float): distance from centre to side edge
		height (float): height of extrusion
		N (int, optional): Number of sides. Defaults to 3.
		rpy (array-like, optional): Roll, pitch and yaw (in that order). Defaults to [0,0,0].
		reverse_normals (bool, optional): If true, normals point inward. Keep true for obstacles, false for rooms.
			Defaults to False.

	Returns:
		list of 2D Nx3 numpy matrices (N>2): Each 2D numpy matrix is a row-wise list array of 
			coordinates of each vertex of a wall.
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
		wall = np.array([
				lower_points[i], upper_points[i],
				upper_points[i+1], lower_points[i+1]
			])
		wall = wall[::-1] if reverse_normals else wall
		walls.append(wall)

	# last side wall
	wall = np.array([
				lower_points[N-1], upper_points[N-1],
				upper_points[0],lower_points[0] 
			])
	wall = wall[::-1] if reverse_normals else wall
	walls.append(wall)

	if reverse_normals:
		lower_points = lower_points[::-1]
		upper_points = upper_points[::-1]

	# lower and upper walls
	walls.append(np.array(lower_points))
	walls.append(np.array(upper_points[::-1]))

	return walls

def make_cylinder(centre, radius, height, rpy=[0,0,0], N=100):
	return make_polygon(centre, radius, height, N=N, rpy=rpy) 

def fix_plt_axs(plt, xylims, zlims):
	plt.gca().set_xlim3d(left=xylims[0], right=xylims[1])
	plt.gca().set_ylim3d(bottom=xylims[0], top=xylims[1])
	plt.gca().set_zlim3d(bottom=zlims[0], top=zlims[1])