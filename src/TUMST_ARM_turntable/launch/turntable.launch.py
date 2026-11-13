from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    cfg=Path(get_package_share_directory('soarm_turntable'))/'config/turntable.yaml'
    return LaunchDescription([DeclareLaunchArgument('read_only',default_value='true',choices=['true','false']),Node(package='soarm_turntable',executable='xl330_bridge',parameters=[str(cfg),{'read_only':ParameterValue(LaunchConfiguration('read_only'),value_type=bool)}],output='screen')])
