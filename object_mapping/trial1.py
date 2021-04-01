# %%
import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt 
from scipy import signal 
from scipy.io import wavfile

# %%

"""
1. Setup env
a. Room init
b. Mic and speaker placement
c. Waveform
d. Objects - using make_polygon
2. Simulate
a. Emit sound and record waveforms
b. get RIRs
3. Processing
a. Remove original waveform but keep reflected impulses
b. extract timestampts of impulses
4. Try out algorithm
"""
# %% Setup Env
# Create a room with completely absorbing walls
fs = 16000
material = pra.Material(energy_absorption=1.0, scattering=None)
room_dim = [10, 10, 5]
room = pra.ShoeBox(room_dim, fs=fs, materials=material)
print('Created room.')

# TODO: Add some polygonal objects

# %% Prepare waveform
def square_wave(freq, duty=0.5, amp=1.0, fs=16000, len=1.0):
	t = np.linspace(0, len, int(len*fs))
	sig = signal.square(2 * np.pi * freq * t, duty=duty)
	return amp * sig

num_cycles = 8
sig_freq = 500
waveform = square_wave(sig_freq, amp=10000, fs=fs, len=num_cycles/sig_freq)
# plt.plot(waveform)

# %% Place mic and sound source
zh = 2
source_coord = [3, 3, zh]
mic_locs = np.c_[
	[5.0, 5.0, zh],
	[5.3, 5.0, zh]
]

room.add_source(source_coord, signal=waveform)
room.add_microphone_array(mic_locs)

# %%

room.plot()
plt.show()

# %% Simulate waveforms and IRs

# TODO
