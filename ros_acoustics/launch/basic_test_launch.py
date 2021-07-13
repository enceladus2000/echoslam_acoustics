from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameters_type import SomeParameters

def generate_launch_description():
	srv_parameters = [
		{'fs': 8000},
		{'ray_tracing': 'False'},
	]
	srv_node = Node(
		package='ros_acoustics',
		executable='srv_compute_waveforms.py',
		parameters=srv_parameters,
	)
	cli_node = Node(
		package='ros_acoustics',
		executable='cli_compute_waveforms.py',
	)

	return LaunchDescription([srv_node, cli_node])

