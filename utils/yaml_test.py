import sys

from numpy.core.fromnumeric import sort
sys.path.append('.')
import os 
from utils.gen_utils import *
import numpy as np
import pyroomacoustics as pra
import yaml
from pprint import pprint
from matplotlib import pyplot as plt

outpath = 'utils/room_data/test1.yaml'

def main():
	# dict_file = {
	# 	'ray_tracing': True,
	# 	'max_order': 4,
	# 	'walls': [
	# 		{
	# 			'id': 0, 
	# 			'material': {
	# 				'abs': .3,
	# 				'sc': .6
	# 			},
	# 			'corners': [
	# 				[1.,2.,3.],
	# 				[2.,3.4,4.6],
	# 				[1.,2.,3.],
	# 				[2.,3.4,4.6]
	# 			]
	# 		},
	# 		{
	# 			'id': 1, 
	# 			'corners': [
	# 				[2.5,2.5,3.5],
	# 				[4.3,3.2,4.5],
	# 				[2.5,2.5,3.5],
	# 				[4.3,3.2,3.5]
	# 			]
	# 		},
	# 	]
	# }
	# print(os.getcwd())
	# with open(r'utils/room_data/test1.yaml', 'w') as file:
	# 	documents = yaml.dump(dict_file, file, default_flow_style=False)

	# room = pra.ShoeBox([2, 3, 4], materials=pra.Material(0.2,0.34), air_absorption=True)

	# # dump room into a yaml
	# dump_room(room, outpath)

	# # pretty print room dict
	# rd = create_room_dict(room)
	# pprint(rd, sort_dicts=False)

	##TESTING AREA##
	room2 = load_room(outpath)
	room2.plot()
	plt.show()

	print('Done')

if __name__ == "__main__":
	main()