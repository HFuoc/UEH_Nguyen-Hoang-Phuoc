from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_speed', default_value='0.18'),
        DeclareLaunchArgument('log_dir', default_value='/tmp/crc_results'),
        Node(package='crc_solution', executable='driver', name='crc_driver',
             output='screen', parameters=[{
                 'use_sim_time': True,
                 'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
                 'log_dir': LaunchConfiguration('log_dir')}])])
