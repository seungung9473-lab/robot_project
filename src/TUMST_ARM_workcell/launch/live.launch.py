"""Measured-state motor control UI. No startup motion; vacuum bridge optional."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from soarm4_workcell.core import load,verify_model

def setup(context):
    w=Path(get_package_share_directory('soarm4_workcell'));p=Path(get_package_share_directory('soarm4_moveit_config'));d=Path(get_package_share_directory('soarm4_description'));b=Path(get_package_share_directory('soarm_bridge'));verify_model(load(w/'config/workcell.yaml'),d,p)
    enabled=LaunchConfiguration('allow_execution').perform(context).lower()=='true'
    if not enabled:raise RuntimeError('Use preview.launch.py for offline display; live motor-control UI requires allow_execution:=true')
    params={'robot_description':(d/'urdf/TUMST_ARM.urdf').read_text(),'robot_description_semantic':(p/'config/TUMST_ARM.srdf').read_text(),'robot_description_planning':yaml.safe_load((p/'config/joint_limits.yaml').read_text()),'allow_trajectory_execution':True,'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':yaml.safe_load((p/'config/ompl_planning.yaml').read_text())}
    return [Node(package='soarm4_workcell',executable='reconstructed_models',parameters=[{'session':LaunchConfiguration('model_session')}],condition=IfCondition(LaunchConfiguration('show_reconstructed_models')),output='screen'),Node(package='soarm_bridge',executable='servo_bridge',parameters=[str(b/'config/TUMST_ARM.yaml'),{'read_only':False}],output='screen'),Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':params['robot_description']}],output='screen'),Node(package='soarm_bridge',executable='esp32_bridge',parameters=[str(b/'config/TUMST_ARM.yaml')],condition=IfCondition(LaunchConfiguration('start_vacuum_bridge')),output='screen'),IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'launch/hardware.launch.py')),launch_arguments={'allow_execution':'true','rviz':'false','use_vacuum_sensor':LaunchConfiguration('use_vacuum_sensor')}.items()),Node(package='rviz2',executable='rviz2',name='hardware_moveit_rviz',arguments=['-d',str(w/'rviz/workcell_hardware.rviz')],parameters=[params],output='screen')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('allow_execution',default_value='false'),DeclareLaunchArgument('show_reconstructed_models',default_value='true'),DeclareLaunchArgument('model_session',default_value='latest'),DeclareLaunchArgument('start_vacuum_bridge',default_value='true'),DeclareLaunchArgument('use_vacuum_sensor',default_value='true',choices=['true','false']),OpaqueFunction(function=setup)])
