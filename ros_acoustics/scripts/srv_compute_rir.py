#! /usr/bin/env python3
from ros_acoustics.srv import ComputeRIR

import pyroomacoustics as pra
import numpy as np
import rclpy
from rclpy.node import Node

class ComputeRIRService(Node):
    # TODO: Docstring
    def __init__(self):
        super().__init__('compute_rir_service')
        self.srv = self.create_service(ComputeRIR, 'compute_rir', self.compute_rir_callback)

    def compute_rir_callback(self, request, response):
        room = pra.ShoeBox([5, 5, 2], fs=8000, max_order=1)
        room.add_source([request.source_pos.x, request.source_pos.y, request.source_pos.z])
        room.add_microphone([request.mic_pos.x, request.mic_pos.y, request.mic_pos.z])
        room.compute_rir()

        response.rir.array = list(room.rir[0][0])
        return response
    
def main(args=None):
    rclpy.init(args=args)

    compute_rir_service = ComputeRIRService()

    rclpy.spin(compute_rir_service)

if __name__ == '__main__':
    main()