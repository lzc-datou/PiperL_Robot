"""Opt-in Domain94 sim with real MoveIt IK delayed by a test-only 45ms proxy.

Run with PIPERL_ROS_DOMAIN_ID=94 and scripts/piperl/run.sh. No driver/SDK/CAN
action is permitted. Both JointJog input and measured simulated joints must
move before MOUSE_UP; a diagnostic Twist by itself cannot pass this test.
"""
import argparse
from collections import Counter
from copy import deepcopy
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests.servo_zeroedge_mock_real import ALLOWED, process_records, same_process, unclean_exits

DOMAIN = '94'
OUTPUT = ROOT / 'outputs/servo_tracking/validation'


def build_delayed_sim(config_file):
    from launch import LaunchContext
    from launch.actions import RegisterEventHandler, EmitEvent, TimerAction, LogInfo
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    spec = importlib.util.spec_from_file_location('production_servo_launch_for_delayed_ik',
                                                 ROOT / 'scripts/task/servo_tracking/task.launch.py')
    production = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(production)
    context = LaunchContext()
    context.launch_configurations.update(mode='sim', rviz='false', config=str(config_file))

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    nodes, identities, plugins = [], [], []
    for action in production.build(context):
        if isinstance(action, Node):
            identity = (text(action.node_package), text(action.node_executable))
            assert identity in ALLOWED, f'Refusing unexpected process: {identity}'
            assert identity[0] != 'agx_arm_ctrl'
            parameters = evaluate_parameters(context, getattr(action, '_Node__parameters'))
            if identity == ('piper_servo_tracking', 'servo_backend'):
                # A test-only replacement with the production evaluated params.
                # Only this Backend client is remapped; Guard/MoveGroup stay native.
                action = Node(package=identity[0], executable=identity[1], name='servo_backend',
                    namespace='/piper_servo', parameters=list(parameters), output='screen',
                    remappings=[('/tf', '/piper_servo/tf'), ('/tf_static', '/piper_servo/tf_static'),
                                ('compute_ik', '/piper_servo/delayed_compute_ik')])
            if identity == ('controller_manager', 'ros2_control_node'):
                descriptions = [item['robot_description'] for item in parameters
                                if isinstance(item, dict) and 'robot_description' in item]
                assert len(descriptions) == 1
                plugins = [item.text.strip() for item in ET.fromstring(descriptions[0]).findall(
                    './/ros2_control/hardware/plugin')]
            nodes.append(action)
            identities.append(identity)
        else:
            assert isinstance(action, (RegisterEventHandler, LogInfo)), type(action)
    assert set(identities) == ALLOWED
    assert plugins == ['mock_components/GenericSystem'], plugins
    # Original handlers refer to the original Backend action; rebuild only for
    # these audited process objects, excluding expected spawner completion.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=action, on_exit=[
        TimerAction(period=2., actions=[EmitEvent(event=Shutdown(reason='owned delayed-IK peer exited'))])]))
        for action in nodes if text(action.node_executable) != 'spawner']
    return handlers + nodes, dict(mode='sim', sdk_or_can_started=False,
        final_processes=identities, hardware_plugins=plugins,
        backend_only_remap='compute_ik -> delayed_compute_ik')


def stack_main(config_file):
    assert os.environ.get('ROS_DOMAIN_ID') == DOMAIN
    from launch import LaunchDescription, LaunchService
    actions, _ = build_delayed_sim(config_file)
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


def main(max_first_motion_s=1., algorithm=None, require_observer_valid=False):
    assert os.environ.get('ROS_DOMAIN_ID') == DOMAIN, 'Fixed isolated Domain94 required'
    assert np.isfinite(max_first_motion_s) and 0 < max_first_motion_s <= 2.
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, Float64
    from std_srvs.srv import Trigger
    from control_msgs.msg import JointJog, JointTrajectoryControllerState
    from trajectory_msgs.msg import JointTrajectory
    from controller_manager_msgs.srv import ListControllers
    from moveit_msgs.msg import ServoStatus
    from moveit_msgs.srv import GetPositionIK
    from rcl_interfaces.srv import GetParameters
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    from scripts.task.servo_tracking.core import JOINTS, RobotModel, load_config
    from scripts.task.servo_tracking.node import pose_message
    import yaml

    OUTPUT.mkdir(parents=True, exist_ok=True)
    lock = open('/tmp/servo_live_drag_domain94.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    report = dict(simulation_only=True, hardware_exercised=False, domain=94, passed=False,
                  mouse_hz=60, ik_minimum_response_delay_ms=45,
                  first_motion_deadline_s=max_first_motion_s)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)

    class Probe(Node):
        def __init__(self):
            super().__init__('servo_delayed_ik_test')
            self.status, self.guard, self.q = {}, {}, []
            self.jogs, self.codes, self.states, self.ik_delays = [], [], [], []
            self.status_samples, self.jtc_samples = [], 0
            self.source_ages = dict(servo=[], controller=[])
            self.publish_panel = self.create_publisher(Float64, '/piper_servo/panel/heartbeat', 1)
            self.marker = self.create_publisher(Feedback, '/piper_servo/target/feedback', 10)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/piper_servo/status', self.on_status, qos)
            self.create_subscription(String, '/piper_servo/guard/status', self.on_guard, qos)
            self.create_subscription(JointState, '/piper_servo/commanded_joint_states', self.on_joints, 1)
            self.create_subscription(JointJog, '/piper_servo/servo_node/delta_joint_cmds',
                lambda message: self.jogs.append((time.monotonic(), list(message.velocities))), 100)
            self.create_subscription(JointTrajectoryControllerState,
                '/piper_servo/arm_controller/controller_state', self.on_jtc, 10)
            self.create_subscription(JointTrajectory, '/piper_servo/servo/trajectory',
                                     self.on_servo_trajectory, 100)
            self.create_subscription(ServoStatus, '/piper_servo/servo_node/status',
                lambda message: self.codes.append((time.monotonic(), int(message.code), message.message)), 100)
            group = ReentrantCallbackGroup()
            self.upstream = self.create_client(GetPositionIK, '/piper_servo/compute_ik', callback_group=group)
            self.create_service(GetPositionIK, '/piper_servo/delayed_compute_ik', self.delayed_ik,
                                callback_group=group)

        def on_joints(self, message):
            self.q[:] = [message.position[message.name.index(name)] for name in JOINTS]

        def on_status(self, message):
            self.status.update(json.loads(message.data))
            self.status_samples.append((time.monotonic(), dict(self.status)))

        def on_jtc(self, message):
            self.jtc_samples += 1
            self.source_ages['controller'].append(time.time()-message.header.stamp.sec-
                                                  message.header.stamp.nanosec*1e-9)

        def on_servo_trajectory(self, message):
            self.source_ages['servo'].append(time.time()-message.header.stamp.sec-
                                             message.header.stamp.nanosec*1e-9)

        def on_guard(self, message):
            self.guard.update(json.loads(message.data))
            value = (self.guard.get('state'), self.guard.get('purpose', ''))
            if not self.states or self.states[-1][1:] != value:
                self.states.append((time.monotonic(), *value))

        def delayed_ik(self, request, response):
            started = time.monotonic()
            ready = threading.Event()
            future = self.upstream.call_async(request)
            future.add_done_callback(lambda ignored: ready.set())
            if not ready.wait(.25):
                response.error_code.val = -6
                return response
            # Delay the checked official result, not a fabricated solution.
            time.sleep(max(0., .045-(time.monotonic()-started)))
            self.ik_delays.append(time.monotonic()-started)
            return future.result()

    node = Probe()
    executor.add_node(node)
    process, stream, owned = None, None, {}
    suffix = '_pi' if algorithm == 'pi' else ''
    log = OUTPUT / f'live_drag_delayed_ik{suffix}.stack.log'

    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            node.publish_panel.publish(Float64(data=time.time()))
            executor.spin_once(timeout_sec=.002)

    def wait(predicate, timeout=10., label='condition'):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            spin(.01)
            if predicate():
                return
        raise AssertionError(f'{label}: status={node.status}; guard={node.guard}')

    def call(name, kind, request, timeout=10.):
        client = node.create_client(kind, '/piper_servo/'+name)
        try:
            wait(client.service_is_ready, timeout, name+' discovery')
            future = client.call_async(request)
            wait(future.done, timeout, name+' response')
            return future.result()
        finally:
            node.destroy_client(client)

    def command(name):
        result = call(name, Trigger, Trigger.Request())
        assert result.success, result.message

    def event(kind, pose):
        message = Feedback()
        message.header.frame_id = 'base_link'
        message.header.stamp = node.get_clock().now().to_msg()
        message.marker_name, message.client_id, message.control_name = 'tcp_target', 'delayed_ik_test', 'move_free'
        message.event_type, message.pose = kind, pose_message(pose)
        node.marker.publish(message)

    try:
        spin(1.)
        others = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
                  if (name, ns) != (node.get_name(), node.get_namespace())]
        assert not others, f'Domain94 not empty: {others}'
        with tempfile.TemporaryDirectory(prefix='servo_delayed_ik_') as temporary:
            config = load_config(ROOT / 'scripts/task/servo_tracking/config/config.yaml')
            if algorithm is not None:
                control = deepcopy(config['control'])
                control['algorithm'] = algorithm
                control_file = Path(temporary)/'control.yaml'
                control_file.write_text(yaml.safe_dump(control))
                config['control_config'] = str(control_file)
            config.update(workspace_samples=4096, workspace_resolution=.06)
            report['algorithm'] = algorithm or config['control']['algorithm']
            report['controller_sync_settle_time_s'] = config.get('controller_sync_settle_time', .3)
            config_file = Path(temporary)/'config.yaml'
            config_file.write_text(yaml.safe_dump(config))
            _, report['launch_audit'] = build_delayed_sim(config_file)
            stream = log.open('w')
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--stack', str(config_file)],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            wait(lambda: node.status.get('mode') == node.guard.get('mode') == 'sim' and node.q and
                 node.status.get('target_revision', 0) > 0, timeout=40., label='production sim ready')
            assert node.status['control_algorithm'] == report['algorithm'], node.status
            deadline = time.monotonic()+20.
            while True:
                controllers = call('controller_manager/list_controllers', ListControllers,
                                   ListControllers.Request()).controller
                if any(item.name == 'arm_controller' and item.state == 'active' for item in controllers):
                    break
                assert time.monotonic() < deadline
                spin(.1)
            wait(lambda: node.count_subscribers('/piper_servo/arm_controller/joint_trajectory') == 1,
                 label='JTC input discovery')
            wait(node.upstream.service_is_ready, label='official IK discovery')
            values = call('move_group/get_parameters', GetParameters,
                          GetParameters.Request(names=['robot_description'])).values
            model = RobotModel(values[0].string_value)
            spin(1.1)  # Expire InteractiveMarkerServer's initial empty-client lease.
            assert not any('agx_arm_ctrl' in name for name, _ in node.get_node_names_and_namespaces())
            command('start')
            wait(lambda: node.guard.get('state') == 'holding', label='initial settled session')
            initial_q = np.asarray(node.q).copy()
            initial_pose = model.pose(initial_q)
            event(Feedback.MOUSE_DOWN, initial_pose)
            started = time.monotonic()
            jog_start = len(node.jogs)
            code_start = len(node.codes)
            first_joint_motion = None
            final_pose = initial_pose
            for index in range(120):
                due = started+index/60.
                if time.monotonic() < due:
                    spin(due-time.monotonic())
                goal_q = initial_q.copy()
                goal_q[0] += .01 + (index+1)*.0005
                final_pose = model.pose(goal_q)
                event(Feedback.POSE_UPDATE, final_pose)
                spin(.001)
                if first_joint_motion is None and np.max(np.abs(np.asarray(node.q)-initial_q)) > .002:
                    first_joint_motion = time.monotonic()-started
                assert node.status['active'], node.status
            spin(max(0., started+2.-time.monotonic()))
            # Assertions deliberately precede MOUSE_UP.
            movement = float(np.max(np.abs(np.asarray(node.q)-initial_q)))
            nonzero = [(stamp, velocity) for stamp, velocity in node.jogs[jog_start:]
                       if np.linalg.norm(velocity) > 1e-5]
            report['before_mouse_release'] = dict(first_joint_motion_s=first_joint_motion,
                first_nonzero_joint_jog_s=nonzero[0][0]-started if nonzero else None,
                nonzero_joint_jog_messages=len(nonzero), max_joint_displacement_rad=movement,
                active=node.status['active'], guard_state=node.guard['state'], updates=120)
            if report['algorithm'] == 'pi':
                # A controlled constant *following* offset exercises stable
                # execution feedback. The goal is .03rad ahead of measured J1;
                # this is not a fixed world target or a frozen physical plant.
                bias_started = time.monotonic()
                for index in range(180):
                    due = bias_started+index/60.
                    if time.monotonic() < due:
                        spin(due-time.monotonic())
                    goal_q = np.asarray(node.q).copy()
                    goal_q[0] += .03
                    model.check_joints(goal_q, config['joint_margin'])
                    final_pose = model.pose(goal_q)
                    event(Feedback.POSE_UPDATE, final_pose)
                    spin(.001)
                    assert node.status['active'], node.status
                report['constant_following_offset'] = dict(joint1_offset_rad=.03,
                    duration_s=time.monotonic()-bias_started, fixed_world_target=False,
                    frozen_plant=False)
            report['running_execution_feedback'] = sorted({sample.get('execution_feedback', '')
                for stamp, sample in node.status_samples if stamp >= started and
                sample.get('active') and sample.get('guard_state') == 'running'})
            report['running_antiwindup_state'] = sorted({sample.get('antiwindup_state', '')
                for stamp, sample in node.status_samples if stamp >= started and
                sample.get('active') and sample.get('guard_state') == 'running'})
            report['running_execution_feedback_counts'] = dict(Counter(sample.get('execution_feedback', '')
                for stamp, sample in node.status_samples if stamp >= started and
                sample.get('active') and sample.get('guard_state') == 'running'))
            report['running_antiwindup_state_counts'] = dict(Counter(sample.get('antiwindup_state', '')
                for stamp, sample in node.status_samples if stamp >= started and
                sample.get('active') and sample.get('guard_state') == 'running'))
            report['jtc_controller_state_samples'] = node.jtc_samples
            report['source_stamp_age_ms'] = {stage: dict(samples=len(ages),
                minimum=min(ages)*1000, maximum=max(ages)*1000,
                median=float(np.median(ages))*1000,
                future_stamps=sum(age < 0 for age in ages))
                for stage, ages in node.source_ages.items() if ages}
            report['ik_response_delay_ms'] = dict(samples=len(node.ik_delays),
                minimum=min(node.ik_delays)*1000, maximum=max(node.ik_delays)*1000)
            assert nonzero, 'No actual JointJog input was observed during the drag'
            # Includes the real Guard's JTC synchronization barrier and a 2mrad
            # measurable displacement. UT separately bounds IK/dispatch timing.
            assert first_joint_motion is not None and first_joint_motion < max_first_motion_s, first_joint_motion
            assert movement > .025, movement
            assert node.guard['state'] == 'running', node.guard
            assert not any(code in (2, 5, 6) for _, code, _ in node.codes[code_start:]), node.codes[code_start:]
            if require_observer_valid or report['algorithm'] == 'pi':
                assert {'execution_unlimited', 'execution_limited'} & set(report['running_execution_feedback']), report
            if report['algorithm'] == 'pi':
                assert {'normal', 'tracking'} & set(report['running_antiwindup_state']), report
            event(Feedback.MOUSE_UP, final_pose)
            wait(lambda: node.guard.get('state') == 'holding', label='released final goal settles')
            assert node.status['active']
            report['released_session_active'] = True
            report['passed'] = True
    except Exception as exc:
        report['error'], report['traceback'] = repr(exc), traceback.format_exc()
    finally:
        if process is not None:
            owned = process_records(process.pid)
            try:
                command('stop')
            except Exception:
                pass
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            until = time.monotonic()+12.
            while process.poll() is None and time.monotonic() < until:
                spin(.05)
            for sig in (signal.SIGTERM, signal.SIGKILL):
                alive = [pid for pid, started in owned.items() if same_process(pid, started)]
                if not alive:
                    break
                for pid in alive:
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
                spin(.5)
            process.wait(timeout=3.)
        if stream is not None:
            stream.close()
        remaining = [pid for pid, started in owned.items() if same_process(pid, started)]
        report['cleanup'] = dict(remaining=remaining, owned_pids=list(owned), unclean_exits=unclean_exits(log))
        if remaining:
            report['passed'] = False
        report['last_status'], report['last_guard'], report['guard_states'] = node.status, node.guard, node.states
        report['status_state_history'] = [(stamp, sample.get('state'), sample.get('reachability'),
            sample.get('execution_feedback')) for stamp, sample in node.status_samples]
        suffix = '_p_braking' if algorithm == 'p' else '_pi' if algorithm == 'pi' else ''
        (OUTPUT/f'live_drag_delayed_ik{suffix}_integration.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        rclpy.shutdown()
        lock.close()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--stack')
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--max-first-motion-s', type=float, default=1.)
    parser.add_argument('--algorithm', choices=['p', 'feedforward_p', 'pi'])
    parser.add_argument('--require-observer-valid', action='store_true')
    args = parser.parse_args()
    if args.stack:
        raise SystemExit(stack_main(args.stack))
    assert args.run, 'Explicit --run required'
    raise SystemExit(main(args.max_first_motion_s, args.algorithm, args.require_observer_valid))
