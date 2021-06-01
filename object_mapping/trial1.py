"""
First test of rudimentary obstacle localisation algorithm
using acoustic echoes
"""

import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.io import wavfile
import os

def main():
	# create anechoic room
	wall_material = pra.Material(energy_absorption=0.1, scattering=None)
	room_dim = [10, 10, 5]
	room = pra.ShoeBox(room_dim, 
						fs=16000,
						materials=wall_material,
						max_order=3,
						ray_tracing=True,
						air_absorption=True,
					)
	print('Created room.')

	# TODO: add polygonal objects

	# prepare waveform
	num_cycles = 8
	sig_freq = 500
	waveform = square_wave(sig_freq, amp=10000, fs=fs, len=num_cycles/sig_freq)
	# plt.plot(waveform)

	# add sources and mics
	zh = 2
	source_coord = [3, 3, zh]
	mic_locs = np.c_[
		[5.0, 5.0, zh],
		[5.3, 5.0, zh]
	]
	room.add_source(source_coord, signal=waveform)
	room.add_microphone_array(mic_locs)
	print('Added source and mics.')

	# visualise room
	# room.plot()
	# plt.show() 

	# Simulated RIRs
	room.compute_rir()
	room.plot_rir()
	plt.show()

	print('Done, exiting...')


def square_wave(freq, duty=0.5, amp=1.0, fs=16000, len=1.0):
	t = np.linspace(0, len, int(len*fs))
	sig = signal.square(2 * np.pi * freq * t, duty=duty)
	return amp * sig

if __name__ == "__main__":
	main()