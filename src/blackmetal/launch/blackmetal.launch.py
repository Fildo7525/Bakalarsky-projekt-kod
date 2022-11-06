import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def getConfigFileLocation(file):
	return os.path.join(
		get_package_share_directory("blackmetal"),
		"config",
		file
	)

def generate_launch_description():
	config_blackmetal = getConfigFileLocation("blackmetal.yaml")
	config_log = getConfigFileLocation("bm_logger.yaml")

	return LaunchDescription([
		Node(
			package='blackmetal',
			executable='blackmetal',
			output='screen',
			parameters = [config_blackmetal],
			emulate_tty=True
		),
		Node(
			package='blackmetal',
			executable='log_server',
			output='screen',
			parameters = [config_log],
			emulate_tty=True
		)
	])

