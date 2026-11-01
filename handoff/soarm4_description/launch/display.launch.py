import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory('soarm4_description')
    desc = open(os.path.join(share, 'urdf', 'soarm4.urdf')).read()
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        # gui:=true  -> sliders (no robot needed)
        # gui:=false -> follow /joint_states from soarm_bridge (real robot); run bringup with use_rsp:=false
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'view.rviz')]),
    ])
