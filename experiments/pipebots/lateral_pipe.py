import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra
import time

def main():
	fs = 16000

	# define the polygonal floor of the pipe
	pol = np.array([
		[0,0], [10,0], [10,-7], [12,-7], [12,9], [10,9], [10,2], [0,2]
	]).T
	room_material = pra.Material(energy_absorption=0.5, scattering=None)

	room = pra.Room.from_corners(
		pol,
		fs=fs,
		max_order=5,
		ray_tracing=True,
		air_absorption=True,
	)
	# create 3D pipe by extruding it 
	height = 2
	room.extrude(height, materials=room_material)

	# TODO: set ray tracing params

	robot_x = 3
	room.add_source([robot_x, 1, 1])
	room.add_microphone([robot_x+0.1, 1, 1])

	# TODO: time the simulation
	# room.image_source_model()
	# room.ray_tracing()
	room.compute_rir()

	# plot RIR according to distance of object reflecting
	impulses = room.rir[0][0]
	dists = np.linspace(0, len(impulses)*343/fs/2, len(impulses))
	plt.plot(dists, impulses)
	plt.xlabel('Distance')
	plt.show()

	# show room
	room.plot(img_order=5)
	plt.gca().set_xlim3d(left=0, right=16)
	plt.gca().set_ylim3d(bottom=-7, top=9)
	plt.gca().set_zlim3d(bottom=-1, top=15)
	plt.show()

if __name__ == "__main__":
	main()