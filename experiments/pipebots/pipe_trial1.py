import sys
# this is a hack, what's a better way of doing this?
sys.path.append('.')   
from utils import pra_utils 

import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

def main():
	# define room
	room_material = pra.Material(energy_absorption=0.1, scattering=None)
	room_dim = [17, 6, 6]
	room = pra.ShoeBox(room_dim, fs=16000, materials=room_material)

	# define pipe
	pipe_center = np.array(room_dim) / 2
	pipe_faces = pra_utils.make_polygon(pipe_center, 1, 15, 4, [0, 1.57, 0])
	pipe_material = pra.Material(energy_absorption=0.2, scattering=0.1)

	pra_utils.add_obstacle(room, pipe_faces, pipe_material)

	# define mics and sources
	room.add_source([5, 3, 3])
	room.add_microphone([5.05, 3, 3.])

	room.image_source_model()

	room.plot(img_order=1)
	plt.gca().set_xlim3d(left=-1, right=18)
	plt.gca().set_ylim3d(bottom=-1, top=18)
	plt.gca().set_zlim3d(bottom=-1, top=18)
	plt.show()

	room.compute_rir()
	room.plot_rir()
	plt.show()

if __name__ == '__main__':
	main()