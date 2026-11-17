"""Opt-in fake-real integration, fixed isolated Domain 89, NEVER SDK/CAN.

The production launch builder supplies MoveGroup, Servo, Guard, Backend and JTC.
Before LaunchService sees any action, the sole agx_arm_ctrl driver is removed,
and every remaining process must match an explicit package/executable allowlist.
Only this test's FakeIO supplies the real-mode feedback/control service topics.

Run: PIPERL_ROS_DOMAIN_ID=89 bash scripts/piperl/run.sh python \
     tests/servo_zeroedge_mock_real.py --run
This models ideal feedback following, not real actuators, CAN, or a hardware test.
"""
import argparse
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
DOMAIN = '89'
OUTPUT = ROOT / 'outputs/servo_tracking/validation'
REPORT = OUTPUT / 'zeroedge_mock_real.json'
ALLOWED = {
    ('moveit_ros_move_group', 'move_group'),
    ('robot_state_publisher', 'robot_state_publisher'),
    ('controller_manager', 'ros2_control_node'),
    ('controller_manager', 'spawner'),
    ('moveit_servo', 'servo_node'),
    ('piper_servo_tracking', 'servo_guard'),
    ('piper_servo_tracking', 'servo_backend'),
}


def build_driverless_real(config_file):
    from launch import LaunchContext
    from launch.actions import RegisterEventHandler, EmitEvent, TimerAction, LogInfo
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.utilities import perform_substitutions
    from launch.utilities import normalize_to_list_of_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    path = ROOT / 'scripts/task/servo_tracking/task.launch.py'
    spec = importlib.util.spec_from_file_location('production_servo_launch_for_mock_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(mode='real', rviz='false', config=str(config_file))
    built = module.build(context)  # Constructs launch objects; starts no processes.

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    nodes, removed, manifest = [], [], []
    for action in built:
        if isinstance(action, Node):
            identity = (text(action.node_package), text(action.node_executable))
            if identity[0] == 'agx_arm_ctrl':
                removed.append(identity)
                continue
            if identity not in ALLOWED:
                raise AssertionError(f'Unapproved process in test launch: {identity!r}')
            nodes.append(action)
            manifest.append(identity)
        elif not isinstance(action, (RegisterEventHandler, LogInfo)):
            raise AssertionError(f'Unexpected action type: {type(action).__name__}')
    assert removed == [('agx_arm_ctrl', 'agx_arm_ctrl_single')], removed
    assert set(manifest) == ALLOWED, manifest
    assert all(text(action.node_package) != 'agx_arm_ctrl' for action in nodes)
    # The real-mode production stack still uses GenericSystem as its JTC-side
    # sampler. Refuse even the driverless launch if a hardware plugin changes.
    plugins = []
    for action in nodes:
        if (text(action.node_package), text(action.node_executable)) != ('controller_manager', 'ros2_control_node'):
            continue
        parameters = evaluate_parameters(context, getattr(action, '_Node__parameters'))
        descriptions = [parameter['robot_description'] for parameter in parameters
                        if isinstance(parameter, dict) and 'robot_description' in parameter]
        assert len(descriptions) == 1, 'Cannot audit JTC hardware description'
        plugins = [plugin.text.strip() for plugin in ET.fromstring(descriptions[0]).findall(
            './/ros2_control/hardware/plugin')]
    assert plugins == ['mock_components/GenericSystem'], plugins
    # Drop ALL production event handlers, including references to the removed
    # driver. Recreate exit handling solely for this final allowed process list.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=action, on_exit=[
        TimerAction(period=2., actions=[EmitEvent(event=Shutdown(reason='owned mock-real peer exited'))])]))
        for action in nodes if text(action.node_executable) != 'spawner']
    return handlers + nodes, dict(removed_driver=removed, final_processes=manifest,
                                 production_mode='real', no_driver_action=True, hardware_plugins=plugins)


def stack_main(config_file, manifest_path):
    assert os.environ.get('ROS_DOMAIN_ID') == DOMAIN, 'fixed test domain required'
    from launch import LaunchDescription, LaunchService
    actions, manifest = build_driverless_real(config_file)
    Path(manifest_path).write_text(json.dumps(manifest, indent=2))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


def process_records(parent):
    """Capture only descendants of our launcher, with PID start-time identity."""
    records = {}
    proc = Path('/proc')
    for item in proc.iterdir():
        if not item.name.isdecimal():
            continue
        try:
            stat = (item / 'stat').read_text()
            fields = stat[stat.rfind(')') + 2:].split()
            records[int(item.name)] = (int(fields[1]), fields[19])
        except (OSError, ValueError, IndexError):
            continue
    owned = {parent}
    for _ in range(20):
        found = {pid for pid, (ppid, _) in records.items() if ppid in owned}
        if found <= owned:
            break
        owned |= found
    return {pid: records[pid][1] for pid in owned if pid in records}


def same_process(pid, started):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
        fields = stat[stat.rfind(')') + 2:].split()
        return fields[19] == started and fields[0] != 'Z'
    except (OSError, IndexError):
        return False


def unclean_exits(log_path):
    if not log_path.exists():
        return []
    return [dict(process=match[0], pid=int(match[1]), exit_code=int(match[2]))
            for match in re.findall(r'\[([^\]]+)\]: process has died \[pid (\d+), exit code (-?\d+)',
                                    log_path.read_text(errors='replace'))]


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == DOMAIN, 'Use PIPERL_ROS_DOMAIN_ID=89 and the runtime wrapper'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from std_msgs.msg import String, Float64
    from std_srvs.srv import SetBool, Empty, Trigger
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import ListControllers
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    from visualization_msgs.srv import GetInteractiveMarkers
    from scripts.task.servo_tracking.core import JOINTS, RobotModel, pose_error, load_config
    from scripts.task.servo_tracking.node import pose_message, pose_values
    import yaml

    OUTPUT.mkdir(parents=True, exist_ok=True)
    lock = open('/tmp/servo_zeroedge_mock_real_domain89.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    report = dict(mock=True, hardware_exercised=False, sdk_or_can_started=False,
                  domain=int(DOMAIN), passed=False,
                  limitation='Ideal discrete position follower; no physical dynamics, SDK, CAN, or real-arm validation',
                  initial_j2_deg=-.021)
    rclpy.init()

    class FakeIO(Node):
        def __init__(self):
            super().__init__('zeroedge_mock_real_io', namespace='/piper_servo')
            self.q = np.zeros(6)
            self.q[1] = math.radians(-.021)
            self.gate_enabled = False
            self.commands = self.closed_commands = self.holds = self.samples = 0
            self.pre_command_samples = self.pre_command_legal_samples = 0
            self.first_command_raw = None
            self.gate_events = []
            self.motor_enable_requests = []
            self.phase = 'preparation'
            self.phase_stats = {}
            self.error = None
            self.model = None
            self.status, self.guard, self.history = {}, {}, []
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.feedback = self.create_publisher(JointState, 'feedback/joint_states', 1)
            self.motor = self.create_publisher(DiagnosticArray, 'feedback/motor_status', 1)
            self.arm = self.create_publisher(DiagnosticArray, 'feedback/arm_status', 1)
            self.panel = self.create_publisher(Float64, 'panel/heartbeat', 1)
            self.marker = self.create_publisher(Feedback, 'target/feedback', 1)
            self.create_subscription(String, 'status', self.on_status, qos)
            self.create_subscription(String, 'guard/status', self.on_guard, qos)
            self.create_subscription(JointState, 'control/joint_states', self.on_command, 1)
            self.create_service(SetBool, 'control_enable', self.on_gate)
            self.create_service(SetBool, 'enable_agx_arm', self.on_motor_enable)
            self.create_service(Empty, 'emergency_stop', self.on_hold)
            self.create_timer(.005, self.tick_feedback)
            self.create_timer(.05, self.tick_diagnostics)
            self.begin_phase('preparation')

        def begin_phase(self, name, test_model_reset=None):
            assert not self.gate_enabled, 'Test model reset is allowed only with gate closed'
            if test_model_reset is not None:
                self.q = np.asarray(test_model_reset).copy()  # Test-only world state; never a command.
            self.phase = name
            self.phase_stats[name] = dict(initial_raw_joints=self.q.tolist(), feedback_samples=0,
                pre_command_samples=0, pre_command_legal_samples=0, forwarded_commands=0,
                strict_commands=0, first_command_raw=None, test_model_reset=test_model_reset is not None)

        def on_status(self, msg):
            self.status = json.loads(msg.data)

        def on_guard(self, msg):
            self.guard = json.loads(msg.data)
            record = (self.guard['state'], self.guard.get('purpose'), self.guard.get('plan_id'))
            if not self.history or self.history[-1] != record:
                self.history.append(record)

        def on_gate(self, request, response):
            self.gate_enabled = bool(request.data)
            self.gate_events.append(dict(enabled=self.gate_enabled, at=time.monotonic()))
            response.success, response.message = True, 'test-only fake control gate ACK'
            return response

        def on_hold(self, request, response):
            self.holds += 1  # Existing q is latched; no calibration or synthetic legal sample.
            return response

        def on_motor_enable(self, request, response):
            self.motor_enable_requests.append(bool(request.data))
            response.success, response.message = False, 'test forbids automatic motor enabling'
            return response

        def on_command(self, message):
            try:
                q = np.array([message.position[message.name.index(name)] for name in JOINTS])
                assert np.isfinite(q).all()
                if self.model is not None:
                    self.model.check_joints(q)  # Even first forwarded command must be strictly valid.
                if not self.gate_enabled:
                    self.closed_commands += 1
                    return
                if self.first_command_raw is None:
                    self.first_command_raw = self.q.copy()
                stats = self.phase_stats[self.phase]
                if stats['first_command_raw'] is None:
                    stats['first_command_raw'] = self.q.tolist()
                stats['forwarded_commands'] += 1
                stats['strict_commands'] += 1
                self.commands += 1
                self.q = q.copy()
            except Exception as exc:
                self.error = repr(exc)

        def tick_feedback(self):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name, message.position = JOINTS, self.q.tolist()
            self.feedback.publish(message)
            self.panel.publish(Float64(data=time.time()))
            self.samples += 1
            stats = self.phase_stats[self.phase]
            stats['feedback_samples'] += 1
            if not stats['forwarded_commands']:
                stats['pre_command_samples'] += 1
                if self.q[1] >= 0.:
                    stats['pre_command_legal_samples'] += 1
            if not self.commands:
                self.pre_command_samples += 1
                if self.q[1] >= 0.:
                    self.pre_command_legal_samples += 1

        def tick_diagnostics(self):
            stamp = self.get_clock().now().to_msg()
            motor = DiagnosticArray()
            motor.header.stamp = stamp
            motor.status = [DiagnosticStatus(name=name, level=DiagnosticStatus.OK,
                message='test-only enabled', values=[KeyValue(key='enabled', value='true')]) for name in JOINTS]
            arm = DiagnosticArray()
            arm.header.stamp = stamp
            arm.status = [DiagnosticStatus(name='arm', level=DiagnosticStatus.OK, message='test-only normal',
                values=[KeyValue(key=key, value=value) for key, value in
                        [('arm_status', '0'), ('teach_status', '0'), ('ctrl_mode', '1')]])]
            self.motor.publish(motor)
            self.arm.publish(arm)

    node = FakeIO()
    launch_process = None
    launch_log = None
    stack_log_path = OUTPUT / 'zeroedge_mock_real.stack.log'
    owned = {}

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)
            if node.error:
                raise AssertionError(node.error)

    def wait(predicate, timeout=10., label='condition', guard_fault=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            spin(.01)
            if predicate():
                return
            if launch_process is not None and launch_process.poll() is not None:
                raise AssertionError(f'Owned launch exited: {launch_process.returncode}')
            if guard_fault and node.guard.get('state') == 'fault':
                raise AssertionError(f'{label}: production guard fault {node.guard}')
        raise AssertionError(f'{label} timeout: status={node.status}; guard={node.guard}')

    def call(name, kind, request, timeout=10.):
        client = node.create_client(kind, name)
        try:
            wait(client.service_is_ready, timeout=timeout, label=name + ' available')
            future = client.call_async(request)
            wait(future.done, timeout=timeout, label=name + ' response')
            return future.result()
        finally:
            node.destroy_client(client)

    def command(name):
        response = call('/piper_servo/' + name, Trigger, Trigger.Request())
        assert response.success, (name, response.message)
        return response.message

    def marker_pose(pose):
        message = Feedback()
        message.header.frame_id = 'base_link'
        message.header.stamp = node.get_clock().now().to_msg()
        message.client_id, message.marker_name = 'mock_real_test', 'tcp_target'
        message.control_name, message.event_type = 'move_x', Feedback.POSE_UPDATE
        message.pose = pose_message(pose)
        node.marker.publish(message)

    try:
        spin(1.)
        others = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
                  if (name, namespace) != ('zeroedge_mock_real_io', '/piper_servo')]
        assert not others, f'Domain 89 is not empty; refusing launch: {others}'
        with tempfile.TemporaryDirectory(prefix='servo_mock_real_') as temporary:
            temporary = Path(temporary)
            config = load_config(ROOT / 'scripts/task/servo_tracking/config/config.yaml')
            config.update(initial_joints=[0.] * 6, workspace_samples=4096, workspace_resolution=.06)
            config_file = temporary / 'config.yaml'
            config_file.write_text(yaml.safe_dump(config))
            manifest_file = temporary / 'manifest.json'
            # Run the safety audit now as well as in the fresh child process.
            _, manifest = build_driverless_real(config_file)
            report['launch_audit'] = manifest
            launch_log = stack_log_path.open('w')
            environment = os.environ.copy()
            environment['ROS_LOG_DIR'] = str(OUTPUT / 'zeroedge_mock_real_ros_logs')
            launch_process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                '--stack', str(config_file), '--manifest', str(manifest_file)],
                stdout=launch_log, stderr=subprocess.STDOUT, env=environment, start_new_session=True)
            wait(lambda: bool(node.status) and bool(node.guard) and node.guard.get('can_plan'),
                 timeout=40., label='production real-mode stack ready')
            assert node.status['mode'] == node.guard['mode'] == 'real'
            assert node.status.get('control_algorithm') == config['control']['algorithm']
            report['effective_control'] = dict(algorithm=config['control']['algorithm'],
                linear_speed=config['linear_speed'], angular_speed=config['angular_speed'],
                joint_speed=config['joint_speed'], control_source=config['control_config'])
            assert not any('agx_arm_ctrl' in name for name, _ in node.get_node_names_and_namespaces())
            report['child_launch_audit'] = json.loads(manifest_file.read_text())
            response = call('/piper_servo/move_group/get_parameters', GetParameters,
                            GetParameters.Request(names=['robot_description']))
            node.model = RobotModel(response.values[0].string_value)
            deadline = time.monotonic() + 20.
            while True:
                controllers = call('/piper_servo/controller_manager/list_controllers',
                                   ListControllers, ListControllers.Request()).controller
                if any(item.name == 'arm_controller' and item.state == 'active' for item in controllers):
                    break
                assert time.monotonic() < deadline, 'arm_controller failed to activate'
                spin(.1)
            wait(lambda: node.count_subscribers('/piper_servo/arm_controller/joint_trajectory') == 1,
                 label='guard joint output subscriber discovery')
            controller = ActionClient(node, FollowJointTrajectory,
                                      '/piper_servo/arm_controller/follow_joint_trajectory')
            try:
                wait(controller.server_is_ready, label='guard joint action server discovery')
            finally:
                controller.destroy()
            spin(1.)
            assert node.q[1] == math.radians(-.021) and node.pre_command_legal_samples == 0
            command('reset')
            original_pose = node.model.pose(node.q)
            report['initial_feedback_samples'] = node.pre_command_samples
            report['initial_legal_samples'] = node.pre_command_legal_samples
            report['initial_raw_joints'] = node.q.tolist()
            command('start')
            wait(lambda: any(state == 'joint_done' and purpose == 'ready'
                             for state, purpose, _ in node.history),
                 timeout=60., label='automatic ready path', guard_fault=True)
            wait(lambda: not node.status.get('active') and node.guard.get('can_plan'),
                 timeout=10., label='ready finished and control stopped')
            ready = np.asarray(config['ready_joints'])
            assert np.max(np.abs(node.q - ready)) <= .01
            assert node.commands > 10 and node.first_command_raw[1] == math.radians(-.021)
            assert node.pre_command_legal_samples == 0
            targets = call('/piper_servo/target/get_interactive_markers', GetInteractiveMarkers,
                           GetInteractiveMarkers.Request()).markers
            target = next(marker for marker in targets if marker.name == 'tcp_target')
            dp, dr = pose_error(pose_values(target.pose), original_pose)
            assert np.linalg.norm(dp) < 1e-10 and np.linalg.norm(dr) < 1e-10
            report['preparation'] = dict(completed=True, real_branch=True,
                retained_user_target=True, final_joints=node.q.tolist(), forwarded_commands=node.commands,
                planning_reference=node.model.planning_joints(np.asarray(report['initial_raw_joints']),
                    config['zero_feedback_tolerance']).tolist())

            # The next point plan must also start from *persistent* raw zero-edge
            # feedback, not from the preceding regular preparation state.
            assert not node.gate_enabled and node.guard.get('can_plan')
            node.begin_phase('direct_point', test_model_reset=np.asarray(report['initial_raw_joints']))
            spin(.8)
            assert node.q[1] == math.radians(-.021)
            response = call('/piper_servo/control_mode', SetBool, SetBool.Request(data=True))
            assert response.success, response.message
            wait(lambda: node.status.get('control_mode') == 'point', label='single-point mode')
            command('reset')
            goal_q = ready.copy()
            goal_q[0] += .10
            goal_q[4] += .05
            target = node.model.pose(goal_q)
            before = node.q.copy()
            marker_pose(target)
            wait(lambda: node.status.get('reachability') == '可达' and
                 node.status.get('ghost_revision') == node.status.get('target_revision'),
                 label='single-point IK preview')
            command('plan')
            wait(lambda: node.status.get('plan_ready'), timeout=40., label='single-point planning', guard_fault=True)
            spin(.4)
            assert np.max(np.abs(node.q - before)) < 1e-10, 'planning must not move fake hardware'
            point_id = node.status['plan_id']
            command('execute')
            wait(lambda: any(state == 'joint_done' and purpose == 'point' and plan_id == point_id
                             for state, purpose, plan_id in node.history),
                 timeout=40., label='single-point physical-feedback settling', guard_fault=True)
            wait(lambda: node.guard.get('can_plan'), label='point close/hold ACK')
            dp, dr = pose_error(target, node.model.pose(node.q))
            assert np.linalg.norm(dp) < .003 and np.linalg.norm(dr) < .03
            assert not node.gate_enabled
            assert node.phase_stats['direct_point']['pre_command_legal_samples'] == 0
            assert node.phase_stats['direct_point']['first_command_raw'][1] == math.radians(-.021)
            report['single_point'] = dict(completed=True, planning_no_motion=True, explicit_execute=True,
                closed_gate_and_hold_ack=True, position_error_mm=float(np.linalg.norm(dp) * 1000),
                orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))), final_joints=node.q.tolist(),
                initial_raw_joints=node.phase_stats['direct_point']['initial_raw_joints'],
                planning_reference=node.model.planning_joints(
                    np.asarray(node.phase_stats['direct_point']['initial_raw_joints']),
                    config['zero_feedback_tolerance']).tolist(), direct_from_persistent_zero_edge=True)

            node.begin_phase('servo')
            response = call('/piper_servo/control_mode', SetBool, SetBool.Request(data=False))
            assert response.success, response.message
            wait(lambda: node.status.get('control_mode') == 'servo', label='servo mode')
            command('reset')
            servo_goal = node.q.copy()
            servo_goal[0] += .025
            servo_pose = node.model.pose(servo_goal)
            marker_pose(servo_pose)
            wait(lambda: node.status.get('reachability') == '可达' and
                 node.status.get('ghost_revision') == node.status.get('target_revision'),
                 label='servo IK preview')
            command('start')
            wait(lambda: node.guard.get('state') in ('running', 'holding'),
                 timeout=12., label='real-branch Servo running', guard_fault=True)
            wait(lambda: node.guard.get('state') == 'holding', timeout=12.,
                 label='real-branch Servo converged and holding', guard_fault=True)
            dp, dr = pose_error(servo_pose, node.model.pose(node.q))
            assert np.linalg.norm(dp) <= config['hold_position_tolerance'] + 1e-6
            assert np.linalg.norm(dr) <= config['hold_orientation_tolerance'] + 1e-6
            assert node.status['active'] and node.phase_stats['servo']['strict_commands'] > 0
            report['servo'] = dict(completed=True, real_branch=True, final_state=node.guard['state'],
                session_stays_active=True, position_error_mm=float(np.linalg.norm(dp) * 1000),
                orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))), final_joints=node.q.tolist())

            wait(lambda: node.guard.get('can_plan') and not node.gate_enabled,
                 label='settled hold gate ACK before home')
            node.begin_phase('home')
            home_history_start = len(node.history)
            command('home')
            wait(lambda: any(state == 'joint_done' and purpose == 'home'
                             for state, purpose, _ in node.history[home_history_start:]),
                 timeout=60., label='explicit home joint path', guard_fault=True)
            wait(lambda: node.status.get('state') == '已回零' and not node.status.get('active') and
                 not node.status.get('busy'), label='home backend completion')
            assert np.max(np.abs(node.q)) <= .01
            targets = call('/piper_servo/target/get_interactive_markers', GetInteractiveMarkers,
                           GetInteractiveMarkers.Request()).markers
            marker = next(item for item in targets if item.name == 'tcp_target')
            dp, dr = pose_error(pose_values(marker.pose), node.model.pose(node.q))
            assert np.linalg.norm(dp) < 1e-10 and np.linalg.norm(dr) < 1e-10
            assert not node.motor_enable_requests
            assert not node.gate_enabled and node.guard['gate_closed']
            assert not any(state == 'running' for state, _, _ in node.history[home_history_start:])
            gate_after_home = len(node.gate_events)
            held_q = node.q.copy()
            spin(.6)
            assert not node.status['active'] and node.status['state'] == '已回零'
            assert not node.gate_enabled
            assert not any(item['enabled'] for item in node.gate_events[gate_after_home:])
            np.testing.assert_allclose(node.q, held_q, atol=1e-10)
            report['home'] = dict(completed=True, explicit_user_request=True, final_joints=node.q.tolist(),
                max_abs_joint_rad=float(np.max(np.abs(node.q))), target_aligned_to_raw_fk=True,
                position_alignment_error_mm=float(np.linalg.norm(dp)*1000),
                orientation_alignment_error_deg=float(np.rad2deg(np.linalg.norm(dr))),
                motor_auto_enable_requests=0, servo_reopened=False, session_stopped=True,
                closed_gate_and_hold_ack=True, zero_calibration_performed=False)
            report['completed_status'], report['completed_guard'] = dict(node.status), dict(node.guard)
            report['passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
        report['traceback'] = traceback.format_exc()
        print(report['traceback'], file=sys.stderr, flush=True)
    finally:
        if launch_process is not None:
            owned = process_records(launch_process.pid)
            try:
                if launch_process.poll() is None:
                    command('stop')
            except Exception:
                pass
            if launch_process.poll() is None:
                os.killpg(launch_process.pid, signal.SIGINT)
            deadline = time.monotonic() + 12.
            while launch_process.poll() is None and time.monotonic() < deadline:
                try:
                    spin(.05)
                except Exception:
                    break
            for sig in (signal.SIGTERM, signal.SIGKILL):
                alive = [pid for pid, started in owned.items() if same_process(pid, started)]
                if not alive:
                    break
                for pid in alive:
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
                until = time.monotonic() + 1.
                while time.monotonic() < until:
                    try:
                        spin(.02)
                    except Exception:
                        break
            try:
                launch_process.wait(timeout=3.)
            except subprocess.TimeoutExpired as exc:
                report['cleanup_error'] = repr(exc)
                report['passed'] = False
        if launch_log is not None:
            launch_log.close()
        remaining = [pid for pid, started in owned.items() if same_process(pid, started)]
        report['cleanup'] = dict(owned_processes=list(owned), remaining=remaining,
            launch_returncode=launch_process.returncode if launch_process is not None else None,
            unclean_process_exits=unclean_exits(stack_log_path))
        if remaining:
            report['passed'] = False
        report['fake_io'] = dict(feedback_samples=node.samples, forwarded_commands=node.commands,
            ignored_closed_gate_commands=node.closed_commands, hold_service_acks=node.holds,
            gate_events=node.gate_events, initial_legal_samples=node.pre_command_legal_samples,
            motor_enable_requests=node.motor_enable_requests)
        report['guard_history'] = node.history
        report['phase_stats'] = node.phase_stats
        report['last_status'], report['last_guard'] = node.status, node.guard
        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        lock.close()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='Run isolated driverless mock-real validation')
    parser.add_argument('--stack', help=argparse.SUPPRESS)
    parser.add_argument('--manifest', help=argparse.SUPPRESS)
    options = parser.parse_args()
    if options.stack:
        raise SystemExit(stack_main(options.stack, options.manifest))
    if not options.run:
        parser.error('Explicit --run is required; no hardware drivers are ever launched')
    raise SystemExit(main())
