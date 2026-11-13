"""One entry point for the physical feedback bridge and measured-state MoveIt."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def setup(context):
    enabled=LaunchConfiguration('allow_execution').perform(context).lower()=='true'
    bridge=Path(get_package_share_directory('soarm_bridge'))/'launch/bringup.launch.py'
    moveit=Path(get_package_share_directory('soarm4_moveit_config'))/'launch/hardware.launch.py'
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge)),launch_arguments={'read_only':str(not enabled).lower(),'rviz':'false','use_rsp':'true','use_cameras':'false'}.items()),IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit)),launch_arguments={'allow_execution':str(enabled).lower(),'use_vacuum_sensor':LaunchConfiguration('use_vacuum_sensor').perform(context),'rviz':LaunchConfiguration('rviz').perform(context)}.items())]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_vacuum_sensor',default_value='true',choices=['true','false']),DeclareLaunchArgument('allow_execution',default_value='false',choices=['true','false']),DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),OpaqueFunction(function=setup)])
