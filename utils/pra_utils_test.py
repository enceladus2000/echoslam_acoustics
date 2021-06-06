import sys
sys.path.append('.')
from utils.pra_utils import *
import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

def main():
	# define room
	fs=16000
	room_material = pra.Material(energy_absorption=0.96, scattering=None)
	room_faces = make_polygon(
		centre=[0,0,2.5],
		radius=10,
		height=5,
		N=4,
		rpy=[0,0,1.57/2]
	)

	# define obstacle
	obstacle_faces = make_polygon(
		centre=[4,0,2.5],
		radius=2.6,
		height=4,
		N=4,
		rpy=[0,0,0.78]
	)
	obstacle_material = pra.Material(energy_absorption=0.1, scattering=0.1)

	# create room
	walls = []
	walls.extend(create_walls(room_faces, room_material))
	walls.extend(create_walls(obstacle_faces, obstacle_material))

	room = pra.Room(walls, fs=fs, max_order=3, ray_tracing=False, air_absorption=True)

	room.add_source([0, 1, 1.])
	room.add_microphone([0, -1, 1])

	# compute rir
	room.image_source_model()
	room.ray_tracing()
	room.compute_rir()

	# plot rir but as distances
	impulses = room.rir[0][0]
	dists = np.linspace(0, len(impulses)*340/fs/2, len(impulses))
	plt.plot(dists, impulses)
	plt.xlabel('Distance')
	plt.show()

	# show room
	room.plot(img_order=1)
	fix_plt_axs(plt, [-9, 9], [-1, 7])
	plt.show()

def fix_plt_axs(plt, xylims, zlims):
	plt.gca().set_xlim3d(left=xylims[0], right=xylims[1])
	plt.gca().set_ylim3d(bottom=xylims[0], top=xylims[1])
	plt.gca().set_zlim3d(bottom=zlims[0], top=zlims[1])

	
if __name__ == '__main__':
	main()