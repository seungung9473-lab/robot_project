"""Offline planning only: fixed HOME, no hardware/serial/vacuum/trajectory execution."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from soarm4_workcell.core import load,verify_model

def generate_launch_description():
    w=Path(get_package_share_directory('soarm4_workcell'));p=Path(get_package_share_directory('soarm4_moveit_config'));d=Path(get_package_share_directory('soarm4_description'));verify_model(load(w/'config/workcell.yaml'),d,p)
    model={'robot_description':(d/'urdf/TUMST_ARM.urdf').read_text(),'robot_description_semantic':(p/'config/TUMST_ARM.srdf').read_text(),'robot_description_planning':yaml.safe_load((p/'config/joint_limits.yaml').read_text()),'allow_trajectory_execution':False,'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':yaml.safe_load((p/'config/ompl_planning.yaml').read_text())}
    common={'namespace':'soarm4_plan','remappings':[('/tf','tf'),('/tf_static','tf_static')]}
    return LaunchDescription([DeclareLaunchArgument('show_workcell_rviz',default_value='true'),IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'launch/planning.launch.py')),launch_arguments={'rviz':'false','demo_plan':'false'}.items()),Node(package='soarm4_workcell',executable='workcell_scene',parameters=[{'config_file':str(w/'config/workcell.yaml'),'attach_camera_roots':False}],output='screen',**common),Node(package='rviz2',executable='rviz2',name='workcell_preview_rviz',arguments=['-d',str(w/'rviz/workcell.rviz')],parameters=[model],condition=IfCondition(LaunchConfiguration('show_workcell_rviz')),**common)])
