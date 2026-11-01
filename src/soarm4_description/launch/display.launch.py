from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    p=Path(get_package_share_directory('soarm4_description'));m=json.loads((p/'config/model.json').read_text());mode=LaunchConfiguration('mode').perform(context);pose=LaunchConfiguration('pose').perform(context)
    if mode in ('home','zero','rest'):pose=mode
    out=[LogInfo(msg='SO-ARM4 CHECKED v5.7: electronics fixed; robot base +90deg; mode='+mode+'; pose='+pose+'; source='+str(p)),Node(package='robot_state_publisher',executable='robot_state_publisher',namespace='soarm4',remappings=[('/tf','tf'),('/tf_static','tf_static')],parameters=[{'robot_description':(p/'urdf/soarm4.urdf').read_text()}])]
    if mode=='gui':
        initial={}
        for j,v in zip(m['joints'],m['poses'][pose]):
            lo,hi=m['display_ranges'][j['name']]
            initial['zeros.'+j['name']]=max(lo,min(hi,float(v)))
        out.append(Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',namespace='soarm4',arguments=[str(p/'urdf/soarm4_visualization.urdf')],parameters=[initial],remappings=[('robot_description','display_only_robot_description')]))
    else:out.append(Node(package='soarm4_description',executable='pose_publisher.py',namespace='soarm4',parameters=[{'pose':pose}]))
    if LaunchConfiguration('rviz').perform(context)=='true':out.append(Node(package='rviz2',executable='rviz2',namespace='soarm4',remappings=[('/tf','tf'),('/tf_static','tf_static')],arguments=['-d',str(p/'rviz/view.rviz')]))
    return out

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='gui',choices=['gui','home','zero','rest']),DeclareLaunchArgument('pose',default_value='home',choices=['home','zero','rest']),DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),OpaqueFunction(function=setup)])
