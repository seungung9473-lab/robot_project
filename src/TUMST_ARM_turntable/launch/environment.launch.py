"""Environment GUI + optional camera drivers + read-only turntable. No motion."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package=Path(get_package_share_directory('soarm_turntable'));bridge=Path(get_package_share_directory('soarm_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('start_cameras',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('start_turntable',default_value='true',choices=['true','false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge/'launch/dual_d415.launch.py')),condition=IfCondition(LaunchConfiguration('start_cameras'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package/'launch/turntable.launch.py')),launch_arguments={'read_only':'true'}.items(),condition=IfCondition(LaunchConfiguration('start_turntable'))),
        Node(package='soarm_turntable',executable='environment_setup',output='screen')])
