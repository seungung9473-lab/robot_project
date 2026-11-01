from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    d=Path(get_package_share_directory('soarm4_description'));p=Path(get_package_share_directory('soarm4_live'))
    common={'namespace':'soarm4_live','remappings':[('/tf','tf'),('/tf_static','tf_static')]}
    return LaunchDescription([
        DeclareLaunchArgument('port',default_value='/dev/servo_bus'),DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='soarm4_live',executable='live_encoders.py',parameters=[{'port':LaunchConfiguration('port'),'rate_hz':20.}],output='screen',**common),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(d/'urdf/soarm4.urdf').read_text()}],**common),
        Node(package='rviz2',executable='rviz2',name='live_robot_rviz',arguments=['-d',str(p/'rviz/live.rviz')],condition=IfCondition(LaunchConfiguration('rviz')),**common),
    ])
