#!/usr/bin/env python3
from ros_acoustics.srv import ComputeRIR

class BasicRIRClient(Node):
	pass
	# def __init__(self):
	#     super().__init__('basic_rir_client')
	#     self.cli = self.create_client(ComputeRIR, 'compute_rir')
		
	#     while not self.cli.wait_for_service(timeout_sec=1.0):
	#         self.get_logger().info('service unavailable, trying again...')
		
	#     self.req = ComputeRIR.Request()

	# def send_request(self):
	#     self.setRequestSourcePos(1, 2, 0.5)
	#     self.setRe
