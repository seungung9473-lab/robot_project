"""Two D415 cameras only. No robot/motor nodes and no invented camera-to-camera TF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rs=Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py'
    def camera(name,serial,profile,depth):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(rs)),launch_arguments={
            'camera_namespace':'camera','camera_name':name,'serial_no':LaunchConfiguration(serial),
            'enable_color':'true','enable_depth':'true',
            'rgb_camera.color_profile':LaunchConfiguration(profile),'depth_module.depth_profile':LaunchConfiguration(depth),
            'align_depth.enable':'true','pointcloud.enable':'true','publish_tf':'true',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('rgb_profile',default_value='640x480x15'),
        DeclareLaunchArgument('rgb_profile1',default_value=LaunchConfiguration('rgb_profile')),
        DeclareLaunchArgument('rgb_profile2',default_value=LaunchConfiguration('rgb_profile')),
        DeclareLaunchArgument('depth_profile',default_value='640x480x15'),
        DeclareLaunchArgument('depth_profile1',default_value=LaunchConfiguration('depth_profile')),
        DeclareLaunchArgument('depth_profile2',default_value=LaunchConfiguration('depth_profile')),
        DeclareLaunchArgument('serial1',default_value='_017122060095'),
        DeclareLaunchArgument('serial2',default_value='_017122060597'),
        camera('cam1','serial1','rgb_profile1','depth_profile1'),TimerAction(period=3.0,actions=[camera('cam2','serial2','rgb_profile2','depth_profile2')])])
