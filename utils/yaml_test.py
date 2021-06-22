import sys
sys.path.append('.')
import os 
from utils.gen_utils import *
import numpy as np
import pyroomacoustics as pra
import yaml
from pprint import pprint
from matplotlib import pyplot as plt

data_path = 'utils/room_data/'

def main():
	# choose any of the below functions to create room in different scenarious,
	# pretty print them, and dump them into a YAML file
	# empty_room(data_path + 'test1.yaml')	

	# read a room dump file
	room_in = load_room(data_path+'test1.yaml')

	# pretty print
	print('Inputted room:')
	pprint(create_room_dict(room_in))
	
	# plot room
	room_in.plot()
	plt.show()

	print('Done')

# The following methods create Room objects in different scenarios
# and dump them into yaml

def empty_room(outpath):
	room = pra.ShoeBox([2.5, 3, 4], materials=pra.Material(0.2,0.34), air_absorption=True)

	# pretty print room dict
	pprint(create_room_dict(room), sort_dicts=False)

	# dump room into a yaml
	dump_room(room, outpath)
	
if __name__ == "__main__":
	main()