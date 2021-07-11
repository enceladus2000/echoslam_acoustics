#!/usr/bin/env python3
from numpy.lib.index_tricks import RClass
from ros_acoustics.srv import ComputeRIR

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

class BasicRIRClient(Node):
	
	def __init__(self):
		super().__init__('basic_rir_client')
		self.cli = self.create_client(ComputeRIR, 'compute_rir')
		
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service unavailable, trying again...')
		
		self.req = ComputeRIR.Request()

	def send_request(self):
		self.req.source_pos.x = 2.
		self.req.source_pos.y = 2.5
		self.req.source_pos.z = 1.
		self.req.mic_pos.x = 2.5
		self.req.mic_pos.y = 2.5
		self.req.mic_pos.z = 1.

		self.future = self.cli.call_async(self.req)

def main(args=None):
	rclpy.init(args=args)

	rir_client = BasicRIRClient()
	rir_client.send_request()

	while rclpy.ok():
		rclpy.spin_once(rir_client)

		if rir_client.future.done():
			try:
				response = rir_client.future.result()
			except Exception as e:
				rir_client.get_logger().info(
					'Service call failed %r' % (e,))
			else:
				rir_client.get_logger().info('Received response! Plotting...')

				plt.plot(response.rir.array)
				plt.show()

			break

	print('Exiting...')
	rir_client.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	main()
