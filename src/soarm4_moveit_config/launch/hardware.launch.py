"""Use measured joint_states; never start a fake publisher or hardware driver here."""
from pathlib import Path
import hashlib,json,yaml,math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def setup(context):
    p=Path(get_package_share_directory('soarm4_moveit_config'));d=Path(get_package_share_directory('soarm4_description'))
    manifest=json.loads((p/'config/model_manifest.json').read_text())
    if hashlib.sha256((d/'urdf/soarm4.urdf').read_bytes()).hexdigest()!=manifest['urdf_sha256']:raise RuntimeError('URDF changed; update audited configuration')
    for name,digest in manifest['meshes'].items():
        if hashlib.sha256((d/'meshes'/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Mesh changed: '+name)
    enabled=LaunchConfiguration('allow_execution').perform(context).lower()=='true'
    model={'robot_description':(d/'urdf/soarm4.urdf').read_text(),'robot_description_semantic':(p/'config/soarm4.srdf').read_text()}
    params={**model,'robot_description_planning':yaml.safe_load((p/'config/joint_limits.yaml').read_text()),'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':yaml.safe_load((p/'config/ompl_planning.yaml').read_text()),'allow_trajectory_execution':enabled,'publish_robot_description':True,'publish_robot_description_semantic':True,'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True,'monitor_dynamics':False,'trajectory_execution.allowed_start_tolerance':math.radians(1.0),'trajectory_execution.allowed_execution_duration_scaling':2.,'trajectory_execution.allowed_goal_duration_margin':3.,'moveit_manage_controllers':False}
    if enabled:params.update(yaml.safe_load((p/'config/hardware_controllers.yaml').read_text()))
    else:params.update({'moveit_controller_manager':'soarm4_moveit_config/PreviewOnlyControllerManager','disable_capabilities':'move_group/MoveGroupExecuteTrajectoryAction'})
    return [LogInfo(msg='SO-ARM4 measured /joint_states; hardware execution='+str(enabled)),Node(package='soarm4_moveit_config',executable='fine_collision_service.py',output='screen'),Node(package='soarm4_moveit_config',executable='pick_place_manager.py',parameters=[{'allow_execution':enabled,'use_vacuum_sensor':LaunchConfiguration('use_vacuum_sensor').perform(context).lower()=='true','recipe_file':LaunchConfiguration('recipe_file').perform(context)}],output='screen'),Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],output='screen'),Node(package='rviz2',executable='rviz2',name='hardware_moveit_rviz',arguments=['-d',str(p/'config/hardware_moveit.rviz')],parameters=[params],condition=IfCondition(LaunchConfiguration('rviz')))]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_vacuum_sensor',default_value='true',choices=['true','false']),DeclareLaunchArgument('allow_execution',default_value='false'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('recipe_file',default_value=str(Path.home()/'.ros/soarm4/pick_place.json')),OpaqueFunction(function=setup)])
