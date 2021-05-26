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
	centre = np.array([3,0,4])
	height = 3
	radius = 2
	N = 4
	theta = np.pi/4
	obstacle_faces = make_polygon(centre,height,radius,N, theta)
	obstacle_material = pra.Material(energy_absorption=0.2, scattering=0.1)

	add_obstacle(room, obstacle_faces, obstacle_material)

	# define mics and sources
	room.add_source([3, 6, 1.])
	room.add_microphone([3, 3, 1.])

	room.image_source_model()

	room.plot(img_order=1)
	plt.show()

if __name__ == '__main__':
	main()