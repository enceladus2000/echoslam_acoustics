import sys
sys.path.append('.')
import os 
from utils.gen_utils import *
import numpy as np
import pyroomacoustics as pra
import yaml

def main():
	dict_file = {
		'ray_tracing': True,
		'max_order': 4,
		'walls': [
			{'id': 0, 'material': {'absorption': 0.2,'scattering': None}, 'vertices': [[1,2,3.0],[2,3,4]]},
			{'id': 1, 'vertices': [[1,4,1],[2,5,4]]},
		]
	}
	# print(os.getcwd())
	# with open(r'utils/room_data/test1.yaml', 'w') as file:
	# 	documents = yaml.dump(dict_file, file, default_flow_style=False)

	room = pra.ShoeBox([2, 3, 4], materials=pra.Material(0.2,0.34), air_absorption=True)
	dump_room(room, 'utils/room_data/test1.yaml')
	rd = create_room_dict(room)
	print(create_room_dict(room))
	print_room(room)
	print('DOne')

if __name__ == "__main__":
	main()