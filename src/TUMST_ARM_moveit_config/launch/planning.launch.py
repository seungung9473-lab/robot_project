from pathlib import Path
import hashlib,json,yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p=Path(get_package_share_directory('soarm4_moveit_config'))
    d=Path(get_package_share_directory('soarm4_description'))
    manifest=json.loads((p/'config/model_manifest.json').read_text())
    if hashlib.sha256((d/'urdf/TUMST_ARM.urdf').read_bytes()).hexdigest()!=manifest['urdf_sha256']:
        raise RuntimeError('Description URDF changed. Regenerate and validate MoveIt configuration.')
    for name,digest in manifest['meshes'].items():
        if hashlib.sha256((d/'meshes'/name).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Collision mesh changed: '+name+'. Regenerate MoveIt audit snapshot.')
    model={'robot_description':(d/'urdf/TUMST_ARM.urdf').read_text(),'robot_description_semantic':(p/'config/TUMST_ARM.srdf').read_text()}
    planning_limits=yaml.safe_load((p/'config/joint_limits.yaml').read_text())
    planning_limits.update(yaml.safe_load((p/'config/pilz_cartesian_limits.yaml').read_text()))
    # Pilz braking magnitude equals the existing acceleration cap; do not raise limits.
    for limit in planning_limits['joint_limits'].values():
        limit.update(has_deceleration_limits=True,max_deceleration=-float(limit['max_acceleration']))
    params={**model,'robot_description_kinematics':yaml.safe_load((p/'config/kinematics.yaml').read_text()),'robot_description_planning':planning_limits,'planning_pipelines':['ompl','pilz_industrial_motion_planner'],'default_planning_pipeline':'ompl','ompl':yaml.safe_load((p/'config/ompl_planning.yaml').read_text()),'pilz_industrial_motion_planner':yaml.safe_load((p/'config/pilz_industrial_motion_planner_planning.yaml').read_text()),'moveit_controller_manager':'soarm4_moveit_config/PreviewOnlyControllerManager','allow_trajectory_execution':False,'disable_capabilities':'move_group/MoveGroupExecuteTrajectoryAction','publish_robot_description':True,'publish_robot_description_semantic':True,'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True,'monitor_dynamics':False}
    # Private namespace keeps the previous slider GUI from publishing this planning state.
    common={'namespace':'soarm4_plan','remappings':[('/tf','tf'),('/tf_static','tf_static')]}
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('demo_plan',default_value='false'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[model],**common),
        Node(package='soarm4_moveit_config',executable='planning_state.py',output='screen',**common),
        Node(package='soarm4_moveit_config',executable='fine_collision_service.py',output='screen',**common),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],output='screen',**common),
        TimerAction(period=3.,actions=[Node(package='soarm4_moveit_config',executable='plan_joint_goal.py',output='screen',condition=IfCondition(LaunchConfiguration('demo_plan')))]),
        Node(package='rviz2',executable='rviz2',name='moveit_rviz',arguments=['-d',str(p/'config/moveit.rviz')],parameters=[params],condition=IfCondition(LaunchConfiguration('rviz')),**common),
    ])
