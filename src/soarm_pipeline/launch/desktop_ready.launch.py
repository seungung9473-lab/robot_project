"""Prepare drivers, cameras, planning scene and RViz; no goal/torque-ON/home calls."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    enabled=LaunchConfiguration('allow_execution').perform(context).lower()=='true'
    share={name:Path(get_package_share_directory(name)) for name in ['soarm_bridge','soarm4_description','soarm4_moveit_config','soarm4_workcell','soarm_turntable']}
    def inc(path,**args):return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=args.items())
    b=share['soarm_bridge'];d=share['soarm4_description']
    from soarm_bridge.camera_profile import dual_camera_profile
    profile,reason=dual_camera_profile(LaunchConfiguration('camera_fps').perform(context))
    return [LogInfo(msg='D415: '+profile+' · '+reason),Node(package='soarm_bridge',executable='servo_bridge',parameters=[str(b/'config/soarm.yaml'),{'read_only':not enabled}],output='screen'),
            Node(package='soarm_bridge',executable='esp32_bridge',parameters=[str(b/'config/soarm.yaml'),{'read_only':not enabled}],output='screen'),
            Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(d/'urdf/soarm4.urdf').read_text()}],output='screen'),
            inc(share['soarm4_moveit_config']/'launch/hardware.launch.py',allow_execution=str(enabled).lower(),use_vacuum_sensor='true',rviz='true'),
            inc(share['soarm_turntable']/'launch/turntable.launch.py',read_only=str(not enabled).lower()),
            inc(b/'launch/dual_d415.launch.py',rgb_profile=profile,depth_profile=profile,pointcloud='false'),
            inc(share['soarm4_workcell']/'launch/scene.launch.py',attach_camera_roots='true')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('allow_execution',default_value='false',choices=['true','false']),DeclareLaunchArgument('camera_fps',default_value='auto',choices=['auto','6','15']),OpaqueFunction(function=setup)])
