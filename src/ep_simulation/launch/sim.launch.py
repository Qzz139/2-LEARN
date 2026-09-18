import tempfile
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ep_simulation.model import adapt
from ep_simulation.scene import semantic, world_sdf, load_config


# 启动时展开 Xacro，并为本次会话生成 URDF、SRDF 和世界文件。
def setup(context):
    sim = Path(get_package_share_directory('ep_simulation'))
    desc = Path(get_package_share_directory('robomaster_description'))
    moveit = Path(get_package_share_directory('ep_moveit_config'))
    runtime = Path(tempfile.mkdtemp(prefix='ep-simulation-'))
    raw = xacro.process_file(str(desc/'urdf/robomaster_ep.urdf.xacro')).toxml()
    urdf = adapt(raw, sim/'config/controllers.yaml')
    # RViz resource_retriever requires a URI; Gazebo accepts file:// too.
    urdf = urdf.replace('package://robomaster_description', desc.as_uri())
    srdf = semantic(urdf)
    (runtime/'robot.urdf').write_text(urdf)
    (runtime/'robot.srdf').write_text(srdf)
    config_path = LaunchConfiguration('bottle_config').perform(context)
    cfg = load_config(config_path or sim/'config/bottle.json')
    (runtime/'pick.world').write_text(world_sdf(cfg))
    params = {'robot_description': urdf, 'robot_description_semantic': srdf,
              'use_sim_time': True,
              'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
              'allow_trajectory_execution': True,
              'publish_robot_description_semantic': True,
              'planning_scene_monitor_options': {'name': 'planning_scene_monitor',
                  'robot_description': 'robot_description', 'joint_state_topic': '/ep_joint_states',
                  'attached_collision_object_topic': '/attached_collision_object',
                  'publish_planning_scene_topic': '/publish_planning_scene',
                  'monitored_planning_scene_topic': '/monitored_planning_scene'},
              'publish_planning_scene': True, 'publish_geometry_updates': True,
              'publish_state_updates': True, 'publish_transforms_updates': True,
              'trajectory_execution.allowed_execution_duration_scaling': 2.0,
              'trajectory_execution.allowed_goal_duration_margin': 3.0,
              'trajectory_execution.allowed_start_tolerance': 0.03}
    params['ompl'] = yaml.safe_load((moveit/'config/ompl.yaml').read_text())
    # Foxy accepts the default pipeline parameters at top-level as well.
    params.update(params['ompl'])
    params['robot_description_planning'] = yaml.safe_load((moveit/'config/joint_limits.yaml').read_text())
    params.update(yaml.safe_load((moveit/'config/moveit_controllers.yaml').read_text()))
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': urdf, 'use_sim_time': True}],
             remappings=[('/joint_states', '/ep_joint_states')]),
        Node(package='ep_simulation', executable='joint_states.py', output='screen'),
        ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                            '-s', 'libgazebo_ros_factory.so', str(runtime/'pick.world')], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'ep', '-file', str(runtime/'robot.urdf'),
                        '-timeout', '60']),
    ]
    for controller in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']:
        actions.append(TimerAction(period=5.0, actions=[Node(package='controller_manager',
            executable='spawner.py', arguments=[controller, '--controller-manager', '/controller_manager'], output='screen')]))
    actions.append(TimerAction(period=8.0, actions=[Node(package='moveit_ros_move_group',
        executable='move_group', output='screen', parameters=[params],
        remappings=[('/joint_states', '/ep_joint_states')])]))
    if LaunchConfiguration('gui').perform(context).lower() == 'true':
        actions.append(TimerAction(period=4.0, actions=[ExecuteProcess(cmd=['gzclient'], output='screen')]))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(TimerAction(period=10.0, actions=[Node(package='rviz2', executable='rviz2',
            arguments=['-d', str(moveit/'config/ep.rviz')], parameters=[params], output='screen',
            remappings=[('/joint_states', '/ep_joint_states')])]))
    if LaunchConfiguration('run_task').perform(context).lower() == 'true':
        actions.append(TimerAction(period=15.0, actions=[Node(package='ep_simulation',
            executable='pick_demo.py', output='screen', arguments=[
                '--config', str(config_path or sim/'config/bottle.json'),
                '--cycles', LaunchConfiguration('cycles').perform(context),
                '--output', LaunchConfiguration('result_file').perform(context)])]))
    print('EP runtime files:', runtime, flush=True)
    return actions


# 声明任务和可视化开关，延迟到上下文就绪后解析路径与参数。
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_task', default_value='false'),
        DeclareLaunchArgument('cycles', default_value='1'),
        DeclareLaunchArgument('result_file', default_value='work/pick_result.json'),
        DeclareLaunchArgument('bottle_config', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'), OpaqueFunction(function=setup)])
