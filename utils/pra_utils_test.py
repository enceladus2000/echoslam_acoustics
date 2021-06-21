import sys
sys.path.append('.')
from utils.pra_utils import *
import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

show_room = True
show_rir = True 

fs = 16000

def main():
	# choose any room scenario from the methods below
	room = room_with_box()

	# plot rir but as distances
	if show_rir:
		impulses = room.rir[0][0]
		dists = np.linspace(0, len(impulses)*340/fs/2, len(impulses))
		plt.plot(dists, impulses)
		plt.xlabel('Distance')
		plt.show()

	# show room
	if show_room:
		room.plot(img_order=3)
		fix_plt_axs(plt, [-9, 9], [-1, 7])
		plt.show()

# The following methods return Room objects in different scenarios

def empty_room():
	"""returns an empty cuboidal room with source and mic somewhat in center"""
	room_material = pra.Material(energy_absorption=0.6, scattering=None)
	room_faces = make_polygon(
		centre=[0,0,2.5],
		radius=10,
		height=5,
		N=4,
		rpy=[0,0,np.pi/4]
	)

	# create room
	walls = []
	walls.extend(create_walls(room_faces, room_material))

	room = pra.Room(walls, fs=fs, max_order=3, ray_tracing=True, air_absorption=False)

	room.add_source([0, 0, 2.])
	room.add_microphone([0, 0.2, 2.1])

	# compute rir
	room.image_source_model()
	room.ray_tracing()
	room.compute_rir()

	return room 

def room_with_box():
	"""Returns cuboidal room with box"""
	room_material = pra.Material(energy_absorption=0.6, scattering=None)
	room_faces = make_polygon(
		centre=[0,0,2.5],
		radius=10,
		height=5,
		N=4,
		rpy=[0,0,np.pi/4]
	)

	# define obstacle
	obstacle_faces = make_polygon(
		centre=[2.5,0,2.5],
		radius=1.8,
		height=3,
		N=4,
		rpy=[0,0,np.pi/4],
		reverse_normals=True
	)
	obstacle_material = pra.Material(energy_absorption=0.1, scattering=0.1)

	# create room
	walls = []
	walls.extend(create_walls(room_faces, room_material))
	walls.extend(create_walls(obstacle_faces, obstacle_material))

	room = pra.Room(walls, fs=fs, max_order=3, ray_tracing=False, air_absorption=False)

	room.add_source([0, 0, 2.])
	room.add_microphone([0, 0.2, 2.1])

	# compute rir
	room.image_source_model()
	room.compute_rir()

	return room 
	
def empty_diff_walls():
	"""Returns empty room with walls of different materials"""
	# 4 side walls are absorptive
	room_materials = [pra.Material(energy_absorption=0.1, scattering=None)] * 4
	# floor and ceiling are reflective
	room_materials.extend([pra.Material(energy_absorption=0.98, scattering=None)] * 2)
	
	room_faces = make_polygon(
		centre=[0,0,2.5],
		radius=10,
		height=5,
		N=4,
		rpy=[0,0,np.pi/4]
	)

	# create room
	walls = []
	walls.extend(create_walls(room_faces, room_materials))

	room = pra.Room(walls, fs=fs, max_order=3, ray_tracing=False, air_absorption=False)

	room.add_source([-5, 2, 2.])
	room.add_microphone([1, 0, 2.])

	# compute rir
	room.image_source_model()
	room.compute_rir()

	return room 

if __name__ == '__main__':
	main()