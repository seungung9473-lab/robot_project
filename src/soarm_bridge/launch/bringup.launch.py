"""ros2 launch soarm_bridge bringup.launch.py [use_cameras:=true] [use_rsp:=true]"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('soarm_bridge'), 'config', 'soarm.yaml')
    try:
        urdf_path = os.path.join(get_package_share_directory('soarm4_description'), 'urdf', 'soarm4.urdf')
        urdf = open(urdf_path).read()
    except Exception:
        urdf = ''
    use_cams = LaunchConfiguration('use_cameras')
    use_rsp = LaunchConfiguration('use_rsp')
    ld = [
        DeclareLaunchArgument('use_cameras', default_value='false'),
        DeclareLaunchArgument('use_rsp', default_value='true'),
        DeclareLaunchArgument('read_only', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='soarm_bridge', executable='servo_bridge', parameters=[cfg, {'read_only': ParameterValue(LaunchConfiguration('read_only'), value_type=bool)}], output='screen'),
        Node(package='soarm_bridge', executable='esp32_bridge', parameters=[cfg], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], condition=IfCondition(use_rsp), output='screen'),
    ]
    ld.append(Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(get_package_share_directory('soarm_bridge'), 'rviz', 'hardware.rviz')], condition=IfCondition(LaunchConfiguration('rviz'))))
    try:
        rs = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_multi_camera_launch.py')
        ld.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs), condition=IfCondition(use_cams),
            launch_arguments={
                'camera_name1': 'cam1', 'serial_no1': '_017122060095',
                'camera_name2': 'cam2', 'serial_no2': '_017122060597',
                'rgb_camera.color_profile1': '640,480,15', 'depth_module.depth_profile1': '640,480,15',
                'rgb_camera.color_profile2': '640,480,15', 'depth_module.depth_profile2': '640,480,15',
                'rgb_camera.auto_exposure_priority1': 'false', 'rgb_camera.auto_exposure_priority2': 'false',
            }.items()))
    except Exception:
        pass
    return LaunchDescription(ld)
