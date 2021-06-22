import numpy as np
import pyroomacoustics as pra
import yaml
import pprint as pp

from yaml.loader import Loader

def load_room(inpath):
	# pass
	with open(inpath, 'r') as file:
		# get room dict
		rdin = yaml.load(file, Loader=yaml.FullLoader)

	pp.pprint(rdin)
	
	walls = []
	for w in rdin['walls']:
		walls.append(
			pra.wall_factory(
				np.array(w['corners']).T,
				0.12,
				0.1
			)
		)

	room = pra.Room(walls,
					fs=rdin['fs'],
					max_order=rdin['max_order'],
					air_absorption=rdin['air_absorption'],
					ray_tracing=rdin['ray_tracing']	
				)
	return room

def dump_room(room, outpath):
	"""TODO: Docstring"""
	rd = create_room_dict(room)
	with open(outpath, 'w') as file:
		yaml.dump(rd, file, default_flow_style=False, sort_keys=False)
	
	print('Done creating YAML dump.')

def print_room(room):
	"""Nicely prints details ab pra.Room object"""
	pp.pprint(create_room_dict(room), sort_dicts=False)

def create_room_dict(room: pra.Room):
	rd = dict()     # room_dict
	rd['ray_tracing'] = room.simulator_state['rt_needed']
	rd['air_absorption'] = room.simulator_state['air_abs_needed']
	rd['max_order'] = room.max_order
	rd['fs'] = room.fs

	rd['walls'] = []
	for widx, wall in enumerate(room.walls):
		wd = dict() # dict for a single wall
		wd['id'] = widx
		wd['material'] = {
			'absorption': float(wall.absorption[0]),
			'scattering': float(wall.scatter[0]) if wall.scatter > 0 else None
		}
		
		# TODO: How to determine normal is reversed
		# wd['reverse_normal'] = False

		corners = wall.corners.T.tolist()
		wd['corners'] = []
		for corner in corners:
			wd['corners'].append(corner)

		rd['walls'].append(wd)

	return rd