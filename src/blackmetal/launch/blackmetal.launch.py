import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def getConfigFileLocation(file):
	return os.path.join(
		get_package_share_directory("blackmetal"),
		"config",
		file
	)

def generate_launch_description():
	config_blackmetal = getConfigFileLocation("blackmetal.yaml")
	config_log = getConfigFileLocation("bm_logger.yaml")

	teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
	teleop_launch_file = os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')

	joy_config = LaunchConfiguration('joy_config', default='ps3')
	# according to the teleop_twist_joy documentatino the parameter default can be one of: atk3, ps3-holonomic, ps3, xbox, xd3.

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
			executable='bm_logger',
			output='screen',
			parameters = [config_log],
			emulate_tty=True
		),
		Node(
			package='blackmetal',
			executable='bm_position',
			output='screen',
			emulate_tty=True
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(teleop_launch_file),
			launch_arguments={'joy_config': joy_config}.items()
		)
	])

