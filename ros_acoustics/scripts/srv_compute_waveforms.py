#! /usr/bin/env python3
from ros_acoustics.srv import ComputeWaveforms

import pyroomacoustics as pra
import numpy as np
import rclpy
from rclpy.node import Node

class ComputeWaveformsService(Node):
	# TODO: Docstring
	def __init__(self, room_path):
		super().__init__('compute_waveforms_service')
		self.srv = self.create_service(ComputeWaveforms, 'compute_waveforms', self.compute_waveforms_callback)
		
		# TODO: Set these variables using parameter server
		self._room_path = room_path
		# self._room_fs = self.get_parameter('fs').value
		self._room_air_absorption = True
		self._room_ray_tracing = False
		
		self._info_log('Initiated compute waveforms service.')

	def compute_waveforms_callback(self, request, response):
		self._info_log('Received request to compute waveforms.')

		return response

	def _info_log(self, info_msg):
		self.get_logger().info(info_msg)
	
def main(args=None):
	rclpy.init(args=None)

	room_path = "/home/tanmay/Projects/echoslam_acoustics/experiments/pipebots/data/t_pipe.yaml"
	compute_waveforms_service = ComputeWaveformsService(room_path)

	rclpy.spin(compute_waveforms_service)

if __name__ == '__main__':
	main()