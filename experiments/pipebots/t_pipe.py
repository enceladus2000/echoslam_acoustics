import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

import sys
sys.path.append('.')
from utils.gen_utils import *
from utils.pra_utils import *

room_path = 'experiments/pipebots/data/t_pipe.yaml'

def main():
	# load room from yaml dump
	room = load_room(room_path)
	print('Room Details:')
	print_room(room)
	
	fs = room.fs

	# TODO: set ray tracing params

	# TODO: Store source and mic pos in YAML file
	robot_x = 1
	room.add_source([robot_x, 0.15, 0.15])
	room.add_microphone([robot_x+0.1, 0.15, 0.15])

	room.image_source_model()
	room.ray_tracing()
	room.compute_rir()

	# plot RIR according to distance of object reflecting
	impulses = room.rir[0][0]
	dists = np.linspace(0, len(impulses)*343/fs/2, len(impulses))
	plt.plot(dists, impulses)
	plt.xlabel('Distance')
	plt.show()

	# show room
	room.plot(img_order=3)
	fix_plt_axs(plt, [-3, 7], [-1, 9])
	plt.show()

if __name__ == "__main__":
	main()