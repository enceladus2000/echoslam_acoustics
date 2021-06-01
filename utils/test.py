from pra_utils import *
import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

def main():
	# define room
	room_material = pra.Material(energy_absorption=0.5, scattering=None)
	room_dim = [10, 10, 5]
	room = pra.ShoeBox(room_dim, fs=16000, materials=room_material)

	# define obstacle
	centre = np.array([3,3,2.5])
	height = 2
	radius = 1
	N = 100
	obstacle_faces = make_polygon(centre,height,radius,N, [1.57,0,0])
	obstacle_material = pra.Material(energy_absorption=0.2, scattering=0.1)

	add_obstacle(room, obstacle_faces, obstacle_material)

	# define mics and sources
	room.add_source([3, 6, 1.])
	room.add_microphone([3, 3, 1.])

	room.image_source_model()

	room.plot(img_order=1)
	plt.gca().set_xlim3d(left=-2, right=11)
	plt.gca().set_ylim3d(bottom=-2, top=11)
	plt.gca().set_zlim3d(bottom=-2, top=11)
	plt.show()

if __name__ == '__main__':
	main()