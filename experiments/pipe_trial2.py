import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra

def main():
	# define room
	room_material = pra.Material(energy_absorption=0.2, scattering=None)
	room_dim = [15, 2, 2]
	fs = 16000
	m = pra.make_materials(
		ceiling=(0.98, 0.1),
		floor=(0.98, 0.1),
		east=(0.1, 0.1),
		west=(0.1, 0.1),
		north=(0.98, 0.1),
		south=(0.98, 0.1),
	)
	room = pra.ShoeBox(room_dim, 
						fs=fs, 
						materials=m,
						max_order=4,
						ray_tracing=True,
						air_absorption=True
					)

	robot_pos_x = 13
	room.add_source([robot_pos_x, 1, 1.])
	room.add_microphone([robot_pos_x, 1, 1.1])

	room.image_source_model()

	# room.plot(img_order=1)
	# plt.gca().set_xlim3d(left=-2, right=16)
	# plt.gca().set_ylim3d(bottom=-2, top=16)
	# plt.gca().set_zlim3d(bottom=-2, top=16)
	# plt.show()

	room.compute_rir()
	# room.plot_rir()
	# plt.show()

	impulses = room.rir[0][0]
	dists = np.linspace(0, len(impulses)*340/fs/2, len(impulses))
	plt.plot(dists, impulses)
	plt.xlabel('Distance')
	plt.show()


if __name__ == '__main__':
	main()