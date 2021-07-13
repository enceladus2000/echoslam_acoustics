#!/usr/bin/env python3
from ros_acoustics.srv import ComputeWaveforms

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

class BasicComputeWaveformsClient(Node):
	
	def __init__(self):
		super().__init__('basic_waveforms_client')
		self.cli = self.create_client(ComputeWaveforms, 'compute_waveforms_client')
		
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service unavailable, trying again in 1s...')
		
		self.req = ComputeWaveforms.Request()

	def send_request(self):
		self.req.source_pos.x = 2.
		self.req.source_pos.y = 2.5
		self.req.source_pos.z = 1.
		self.req.mic_pos.x = 2.5
		self.req.mic_pos.y = 2.5
		self.req.mic_pos.z = 1.
		self.req.source_wav = [.1, .2, .3, .24]

		self.future = self.cli.call_async(self.req)

def main(args=None):
	rclpy.init(args=None)

	waveforms_client = BasicComputeWaveformsClient()
	waveforms_client.send_request()

	while rclpy.ok():
		rclpy.spin_once(waveforms_client)

		if waveforms_client.future.done():
			try:
				response = waveforms_client.future.result()
			except Exception as e:
				waveforms_client.get_logger().info(
					'Service call failed %r' % (e,))
			else:
				waveforms_client.get_logger().info('Received response! Plotting...')

				plt.plot(response.waveforms[0].array)
				plt.show()

			break

	print('Exiting...')
	waveforms_client.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	main()
