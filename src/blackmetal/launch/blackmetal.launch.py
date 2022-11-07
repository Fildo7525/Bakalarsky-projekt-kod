from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='blackmetal',
			executable='blackmetal',
			output='screen',
			emulate_tty=True
		),
		Node(
			package='blackmetal',
			executable='log_server',
			output='screen',
			emulate_tty=True
		)
	])

