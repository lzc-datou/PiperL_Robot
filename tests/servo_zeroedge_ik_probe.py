"""Read-only IK seed comparison on driverless fake-real Domain 89.

No start/plan/execute calls. Persistent raw J2=-0.021 degrees is never changed.
The child launch is audited by servo_zeroedge_mock_real before execution.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests.servo_zeroedge_mock_real import (
    build_driverless_real, process_records, same_process, unclean_exits,
)

OUTPUT = ROOT / 'outputs/servo_tracking/validation'


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '89', 'Fixed isolated test Domain 89 required'
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from std_msgs.msg import String, Float64
    from std_srvs.srv import SetBool, Empty, Trigger
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPositionIK
    from scripts.task.servo_tracking.core import JOINTS, RobotModel, pose_error
    from scripts.task.servo_tracking.node import pose_message
    import yaml

    lock = open('/tmp/servo_zeroedge_mock_real_domain89.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    report = dict(mock=True, hardware_exercised=False, sdk_or_can_started=False,
                  read_only_ik=True, passed=False, domain=89, results=[])
    rclpy.init()
    node = rclpy.create_node('zeroedge_read_only_ik_probe', namespace='/piper_servo')
    raw = np.array([0., math.radians(-.021), 0., 0., 0., 0.])
    q = raw.copy()
    samples = [0]
    control_messages, gate_events, status, guard = [], [], {}, {}
    feedback = node.create_publisher(JointState, 'feedback/joint_states', 1)
    motor = node.create_publisher(DiagnosticArray, 'feedback/motor_status', 1)
    arm = node.create_publisher(DiagnosticArray, 'feedback/arm_status', 1)
    heartbeat = node.create_publisher(Float64, 'panel/heartbeat', 1)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, 'status', lambda msg: status.update(json.loads(msg.data)), qos)
    node.create_subscription(String, 'guard/status', lambda msg: guard.update(json.loads(msg.data)), qos)
    node.create_subscription(JointState, 'control/joint_states', lambda msg: control_messages.append(list(msg.position)), 1)

    def gate_callback(request, response):
        gate_events.append(bool(request.data))
        response.success, response.message = True, 'test-only gate ACK'
        return response

    node.create_service(SetBool, 'control_enable', gate_callback)
    node.create_service(Empty, 'emergency_stop', lambda request, response: response)

    def publish_feedback():
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name, message.position = JOINTS, q.tolist()
        feedback.publish(message)
        heartbeat.publish(Float64(data=time.time()))
        samples[0] += 1

    def publish_diagnostics():
        stamp = node.get_clock().now().to_msg()
        motors = DiagnosticArray()
        motors.header.stamp = stamp
        motors.status = [DiagnosticStatus(name=name, level=DiagnosticStatus.OK, message='test-only enabled',
            values=[KeyValue(key='enabled', value='true')]) for name in JOINTS]
        arms = DiagnosticArray()
        arms.header.stamp = stamp
        arms.status = [DiagnosticStatus(name='arm', level=DiagnosticStatus.OK, message='test-only normal',
            values=[KeyValue(key=key, value=value) for key, value in
                    [('arm_status', '0'), ('teach_status', '0'), ('ctrl_mode', '1')]])]
        motor.publish(motors)
        arm.publish(arms)

    node.create_timer(.005, publish_feedback)
    node.create_timer(.05, publish_diagnostics)
    process, log = None, None
    owned = {}
    log_path = OUTPUT / 'zeroedge_ik_probe.stack.log'

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005)

    def wait(predicate, timeout=10.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(.01)
            if predicate():
                return
            if process is not None and process.poll() is not None:
                raise AssertionError(f'owned launch exited {process.returncode}')
        raise AssertionError(f'probe wait timeout: {status}; {guard}')

    def call(service, kind, request):
        client = node.create_client(kind, service)
        try:
            wait(client.service_is_ready)
            future = client.call_async(request)
            wait(future.done)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        spin(1.)
        others = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
                  if (name, namespace) != ('zeroedge_read_only_ik_probe', '/piper_servo')]
        assert not others, f'Domain89 not empty: {others}'
        with tempfile.TemporaryDirectory(prefix='servo_ik_probe_') as temporary:
            temporary = Path(temporary)
            config = yaml.safe_load((ROOT / 'scripts/task/servo_tracking/config/config.yaml').read_text())
            config.update(initial_joints=[0.] * 6, workspace_samples=4096, workspace_resolution=.06)
            config_file = temporary / 'config.yaml'
            config_file.write_text(yaml.safe_dump(config))
            _, report['launch_audit'] = build_driverless_real(config_file)
            log = log_path.open('w')
            environment = os.environ.copy()
            environment['ROS_LOG_DIR'] = str(OUTPUT / 'zeroedge_ik_probe_ros_logs')
            process = subprocess.Popen([sys.executable, str(ROOT / 'tests/servo_zeroedge_mock_real.py'),
                '--stack', str(config_file), '--manifest', str(temporary / 'manifest.json')],
                env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait(lambda: status.get('mode') == guard.get('mode') == 'real' and guard.get('can_plan'), 40.)
            response = call('/piper_servo/move_group/get_parameters', GetParameters,
                            GetParameters.Request(names=['robot_description']))
            model = RobotModel(response.values[0].string_value)
            goal_q = np.asarray(config['ready_joints'])
            goal_q[0] += .10
            goal_q[4] += .05
            target = model.pose(goal_q)
            report.update(raw_joints=raw.tolist(), goal_joints=goal_q.tolist(), target_pose=target.tolist())
            seeds = [('raw', raw), ('planning_reference', model.planning_joints(raw,
                         config['zero_feedback_tolerance'])),
                     ('configured_ready', np.asarray(config['ready_joints'])),
                     ('model_middle', (model.lower + model.upper) / 2.)]
            ik_client = node.create_client(GetPositionIK, '/piper_servo/compute_ik')
            wait(ik_client.service_is_ready)
            spin(.2)
            for name, seed in seeds:
                request = GetPositionIK.Request()
                request.ik_request.group_name, request.ik_request.ik_link_name = 'arm', 'tcp_link'
                request.ik_request.avoid_collisions = True
                request.ik_request.robot_state.joint_state.name = JOINTS
                request.ik_request.robot_state.joint_state.position = seed.tolist()
                request.ik_request.pose_stamped.header.frame_id = 'base_link'
                request.ik_request.pose_stamped.header.stamp = node.get_clock().now().to_msg()
                request.ik_request.pose_stamped.pose = pose_message(target)
                request.ik_request.timeout.nanosec = 30_000_000  # Same as production Backend.
                started = time.monotonic()
                future = ik_client.call_async(request)
                wait(future.done)
                response = future.result()
                entry = dict(seed=name, seed_joints=seed.tolist(), error_code=response.error_code.val,
                             elapsed_ms=(time.monotonic()-started)*1000., requested_timeout_ms=30.)
                if response.error_code.val == response.error_code.SUCCESS:
                    js = response.solution.joint_state
                    solution = np.array([js.position[js.name.index(joint)] for joint in JOINTS])
                    dp, dr = pose_error(target, model.pose(solution))
                    entry.update(solution_joints=solution.tolist(), position_error_mm=float(np.linalg.norm(dp)*1000),
                                 orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))))
                    try:
                        model.check_joints(solution, config['joint_margin'])
                        entry['strict_target_margin_valid'] = True
                    except ValueError as exc:
                        entry.update(strict_target_margin_valid=False, bounds_error=str(exc))
                report['results'].append(entry)
                assert not control_messages and not any(gate_events), 'Read-only probe must never authorize control'
                np.testing.assert_array_equal(q, raw)
            node.destroy_client(ik_client)
            report['passed'] = True
    except Exception as exc:
        report['error'], report['traceback'] = repr(exc), traceback.format_exc()
    finally:
        if process is not None:
            owned = process_records(process.pid)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            end = time.monotonic()+12.
            while process.poll() is None and time.monotonic() < end:
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
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired as exc:
                report.update(passed=False, cleanup_error=repr(exc))
        if log is not None:
            log.close()
        remaining = [pid for pid, started in owned.items() if same_process(pid, started)]
        report['cleanup'] = dict(remaining=remaining, unclean_process_exits=unclean_exits(log_path))
        report.update(raw_unchanged=bool(np.array_equal(q, raw)), feedback_samples=samples[0],
                      forwarded_commands=len(control_messages), gate_enable_requests=sum(gate_events))
        if remaining or control_messages or any(gate_events):
            report['passed'] = False
        (OUTPUT / 'zeroedge_mock_real_ik_probe.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        lock.close()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    if not parser.parse_args().run:
        parser.error('Explicit --run required')
    raise SystemExit(main())
