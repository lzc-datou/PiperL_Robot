"""Private task stack: sim never creates an SDK/CAN driver."""
from pathlib import Path
import tempfile
import os
import sys
ROOT_ENV = os.environ.get('PIPERL_ROOT')
if ROOT_ENV and ROOT_ENV not in sys.path: sys.path.insert(0, ROOT_ENV)
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, TimerAction, LogInfo, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from servo_tracking.core import JOINTS, load_config, joint_speed_limits
from scripts.tools.gamepad_control.settings import load_tool_config


def build(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    config_file = LaunchConfiguration('config').perform(context)
    control_file = LaunchConfiguration('control_config', default='').perform(context)
    config = load_config(config_file, control_file)
    gamepad_mode = LaunchConfiguration('gamepad', default='false').perform(context) == 'true'
    gamepad_config = LaunchConfiguration('gamepad_config', default='').perform(context)
    input_source = LaunchConfiguration('gamepad_input_source', default='fixture').perform(context)
    input_device = LaunchConfiguration('gamepad_device', default='').perform(context)
    gamepad_link_verified = LaunchConfiguration('gamepad_link_verified', default='false').perform(context) == 'true'
    gamepad_mapping_verified = LaunchConfiguration('gamepad_mapping_verified', default='false').perform(context) == 'true'
    if gamepad_mode:
        if not gamepad_config:
            gamepad_config = str(Path(os.environ.get('PIPERL_ROOT',''))/'scripts/tools/gamepad_control/config/config.yaml')
        gp = load_tool_config(gamepad_config)
        if mode == 'real' and (input_source != 'evdev' or not gamepad_link_verified or not gamepad_mapping_verified):
            raise ValueError('实机模式只接受已分别验证失联行为与轴映射的 evdev 输入')
        if mode == 'real' and input_source == 'fixture':
            raise ValueError('实机模式禁止输入 fixture')
        if input_source == 'fixture' and mode != 'sim':
            raise ValueError('fixture 仅允许纯仿真')
        config['joint_speed'] = min(float(config['joint_speed']), float(gp['joint_speed']))
        config['joint_acceleration'] = min(float(config['joint_acceleration']), float(gp['joint_acceleration']))
        config['default_speed_scale'] = float(gp['default_speed_scale'])
        effective = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='piper_gamepad_task_', delete=False)
        yaml.safe_dump(config, effective); effective.close(); config_file = effective.name
    control_file = config['control_config']
    if config['control_legacy_overrides']:
        effective = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='piper_servo_control_', delete=False)
        yaml.safe_dump(config['control'], effective)
        effective.close()
        control_file = effective.name
    initial = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='piper_servo_initial_', delete=False)
    yaml.safe_dump({'initial_positions': dict(zip(JOINTS, config['initial_joints']))}, initial)
    initial.close()
    tcp = config['tcp_offset']
    moveit = (MoveItConfigsBuilder('agx_arm', package_name='agx_arm_moveit')
              .robot_description(file_path='config/agx_arm.urdf.xacro', mappings={
                  'arm_type': 'piper_l', 'effector_type': 'none', 'initial_positions_file': initial.name,
                  'tcp_offset_xyz': ' '.join(map(str, tcp[:3])), 'tcp_offset_rpy': ' '.join(map(str, tcp[3:]))})
              .robot_description_semantic(file_path='config/agx_arm.srdf.xacro', mappings={'arm_type': 'piper_l', 'effector_type': 'none'})
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .trajectory_execution(file_path='config/moveit_controllers_none.yaml')
              .to_moveit_configs())
    for name, limit in zip(JOINTS, joint_speed_limits(config)):
        moveit.joint_limits['robot_description_planning']['joint_limits'][name].update(
            has_velocity_limits=True, max_velocity=float(limit),
            has_acceleration_limits=True, max_acceleration=float(config['joint_acceleration']))
    ns = '/piper_servo'
    feedback = ns + ('/feedback/joint_states' if mode == 'real' else '/commanded_joint_states')
    remaps = [('/tf', ns+'/tf'), ('/tf_static', ns+'/tf_static'), ('joint_states', feedback)]
    model_parameters = [moveit.robot_description, moveit.robot_description_semantic,
                        moveit.robot_description_kinematics, moveit.joint_limits]
    nodes = []
    def node(package, executable, name, params, mappings=None, arguments=None):
        action = Node(package=package, executable=executable, name=name, namespace=ns,
                      parameters=params, remappings=remaps+(mappings or []), arguments=arguments or [], output='screen')
        nodes.append(action)
        return action
    # MoveGroup plans only; the watchdog owns the controller's action executor.
    node('moveit_ros_move_group', 'move_group', 'move_group', [moveit.to_dict(), {
        'allow_trajectory_execution': False, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
    }])
    node('robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher', [moveit.robot_description])
    controller_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='piper_servo_controllers_', delete=False)
    yaml.safe_dump({
        ns+'/controller_manager': {'ros__parameters': {'update_rate': 200,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'arm_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        ns+'/arm_controller': {'ros__parameters': {'joints': JOINTS, 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'], 'allow_nonzero_velocity_at_trajectory_end': True}},
    }, controller_file)
    controller_file.close()
    # This mock is a controller-side trajectory sampler, never the real feedback source.
    controller = Node(package='controller_manager', executable='ros2_control_node', name='controller_manager', namespace=ns,
        parameters=[moveit.robot_description, controller_file.name],
        remappings=[('joint_states', ns+'/commanded_joint_states'), ('robot_description', ns+'/robot_description')], output='screen')
    nodes.append(controller)
    spawners = [Node(package='controller_manager', executable='spawner', namespace=ns,
        arguments=[name, '-c', ns+'/controller_manager', '--controller-manager-timeout', '30'], output='screen')
        for name in ('joint_state_broadcaster', 'arm_controller')]
    servo = {
        'move_group_name': 'arm', 'publish_period': 1./config['rate'], 'max_expected_latency': .08,
        'command_in_type': 'speed_units', 'scale': {'linear': config['linear_speed'], 'rotational': config['angular_speed'], 'joint': 1. if gamepad_mode else .3},
        'joint_command_in_topic': ns+'/servo_node/delta_joint_cmds',
        'command_out_type': 'trajectory_msgs/JointTrajectory', 'command_out_topic': ns+'/servo/trajectory',
        'joint_topic': feedback, 'monitored_planning_scene_topic': ns+'/monitored_planning_scene',
        'is_primary_planning_scene_monitor': False, 'check_collisions': True, 'collision_check_rate': 30.,
        'self_collision_proximity_threshold': .01, 'scene_collision_proximity_threshold': .025,
        'joint_limit_margins': [config['joint_margin']], 'incoming_command_timeout': config['command_timeout'],
        # Keep the solved TCP direction through the last bound check. The
        # gamepad guard keeps the session open on JOINT_BOUND, so a fresh
        # inward request can retreat without unlocking or restarting.
        'halt_all_joints_in_joint_mode': True,
        # Native JOINT_JOG skips Cartesian condition-number scaling while the
        # shared official output path retains collisions and joint constraints.
        'publish_joint_positions': True, 'publish_joint_velocities': True,
        'use_smoothing': True, 'smoothing_filter_plugin_name': 'online_signal_smoothing::AccelerationLimitedPlugin',
        'apply_twist_commands_about_ee_frame': True,
    }
    node('moveit_servo', 'servo_node', 'servo_node', model_parameters + [
        {'moveit_servo': servo, 'update_period': 1./config['rate'], 'planning_group_name': 'arm'}])
    if gamepad_mode:
        guard_params = {'config_file': config_file, 'control_config_file': control_file, 'mode': mode,
                        'gamepad_mode': True, 'gamepad_input_timeout_s': gp['input_timeout_s'],
                        'gamepad_gui_timeout_s': gp['gui_timeout_s'],
                        'gamepad_deadzone': gp['deadzone'],
                        'gamepad_stop_settle_window_s': gp['stop_settle_window_s'],
                        'gamepad_stop_joint_span_rad': gp['stop_joint_span_rad'],
                        'require_verified_gamepad_link': True,
                        'require_verified_gamepad_mapping': True}
        node('piper_servo_tracking', 'servo_guard', 'servo_guard', [moveit.robot_description, guard_params])
        node('piper_servo_tracking', 'gamepad_backend', 'gamepad_backend', [moveit.robot_description, {
            'config_file': config_file, 'mode': mode, 'gamepad_config': gamepad_config}])
        if input_source != 'external':
            node('piper_servo_tracking', 'gamepad_input', 'gamepad_input', [{
                'source': input_source, 'device': input_device,
                'link_verified': gamepad_link_verified,
                'mapping_verified': gamepad_mapping_verified}])
    else:
        for executable in ('servo_guard', 'servo_backend'):
            node('piper_servo_tracking', executable, executable, [moveit.robot_description,
                 {'config_file': config_file, 'control_config_file': control_file, 'mode': mode}])
    if mode == 'real':
        node('agx_arm_ctrl', 'agx_arm_ctrl_single', 'agx_arm_ctrl_single_node', [{
            'can_port': config['can_port'], 'arm_type': 'piper_l', 'effector_type': 'none',
            'auto_enable': False, 'control_enabled': False, 'fast_mode': bool(config['fast_mode']),
            'pub_rate': 200, 'tcp_offset': list(map(float, tcp)), 'speed_percent': 0,
        }])
    if LaunchConfiguration('rviz').perform(context) == 'true':
        rviz_config = 'gamepad.rviz' if gamepad_mode else 'servo.rviz'
        node('rviz2', 'rviz2', 'servo_rviz', model_parameters,
             arguments=['-d', str(Path(get_package_share_directory('piper_servo_tracking'))/'config'/rviz_config)])
    # A dead critical node/closed RViz ends the owned stack. run.py requests a stop before teardown.
    # Keep watchdog/driver alive for failure detection and hold before launch tears down peers.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[TimerAction(
        period=2., actions=[EmitEvent(event=Shutdown(reason='Servo task process exited'))])])) for n in nodes]
    control_log = f'Cartesian control source: {config["control_config"]}; algorithm={config["control"]["algorithm"]}'
    if config['control_legacy_overrides']:
        control_log += f'; legacy flat overrides applied, effective snapshot: {control_file}'
    return [LogInfo(msg=control_log)]+handlers+nodes+spawners


def generate_launch_description():
    share = Path(get_package_share_directory('piper_servo_tracking'))
    ros_home = Path(os.environ.get('PIPERL_ROOT',str(Path.cwd())))/'outputs/ros_home'
    (ros_home/'locks').mkdir(parents=True,exist_ok=True)
    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_HOME',value=str(ros_home)),
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('config', default_value=str(share/'config/config.yaml')),
        DeclareLaunchArgument('control_config', default_value=''),
        DeclareLaunchArgument('gamepad', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('gamepad_config', default_value=''),
        DeclareLaunchArgument('gamepad_input_source', default_value='fixture', choices=['evdev','fixture','external']),
        DeclareLaunchArgument('gamepad_device', default_value=''),
        DeclareLaunchArgument('gamepad_link_verified', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('gamepad_mapping_verified', default_value='false', choices=['true','false']),
        OpaqueFunction(function=build),
    ])
