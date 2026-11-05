"""Add workcell to an already-running MoveIt. Never starts hardware drivers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from soarm4_workcell.core import load,verify_model

def generate_launch_description():
    w=Path(get_package_share_directory('soarm4_workcell'));p=Path(get_package_share_directory('soarm4_moveit_config'));d=Path(get_package_share_directory('soarm4_description'));verify_model(load(w/'config/workcell.yaml'),d,p)
    return LaunchDescription([DeclareLaunchArgument('attach_camera_roots',default_value='true',choices=['true','false']),Node(package='soarm4_workcell',executable='workcell_scene',parameters=[{'config_file':str(w/'config/workcell.yaml'),'attach_camera_roots':ParameterValue(LaunchConfiguration('attach_camera_roots'),value_type=bool)}],output='screen')])
