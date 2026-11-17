"""Isolated Domain91 sim: native joint-bound pause and same-goal recovery.

The passive real capture did not record the preceding rotation. The boundary
case therefore starts at its observed post-rotation joints and replays its
fixed-orientation target. Ordinary cases test both manipulation orders.

--baseline proves native code6, a valid IK branch, and an official guarded joint
plan to that same goal. --expect-auto requires recovery with no new target,
start, reset or mode switch. This script owns/cleans its sim stack, never SDK/CAN.
--retarget-recovery additionally cancels/replaces three running recovery goals,
then verifies a new Servo drag and an explicit stop/start ownership handoff.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests.servo_zeroedge_mock_real import process_records, same_process

OUTPUT = ROOT / 'outputs/servo_tracking/validation'
DEFAULT_CASE = OUTPUT / 'passive_rotation_translation_joint4_case_20261113.json'
DEFAULT_IK = OUTPUT / 'passive_rotation_translation_joint4_case_20261113_offline_ik.json'


def run_stack(initial_q, case, options, scenario):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String, Float64
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import ServoStatus, DisplayRobotState
    from moveit_msgs.srv import GetPositionIK
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import ListControllers
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    from visualization_msgs.srv import GetInteractiveMarkers
    from piper_servo_tracking.srv import PlanJointTarget, JointPlanCommand
    from scripts.task.servo_tracking.core import JOINTS, RobotModel, pose_error, load_config
    from scripts.task.servo_tracking.node import pose_message, pose_values
    import yaml

    rclpy.init()
    node = rclpy.create_node('rotation_translation_sim_test')
    status, guard, q = {}, {}, []
    codes, transitions, ghost, foreign_actions = [], [], [], []
    status_trace = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/piper_servo/status', lambda msg: status.update(json.loads(msg.data)), qos)

    def on_guard(msg):
        guard.update(json.loads(msg.data))
        if guard.get('other_action_active'):
            foreign_actions.append(dict(guard))
        value = (guard['state'], guard.get('purpose'), guard.get('plan_id'))
        if not transitions or transitions[-1] != value:
            transitions.append(value)

    def on_status(msg):
        value = dict(code=int(msg.code), message=msg.message, at=time.monotonic(), joints=q[:])
        if not codes or codes[-1]['code'] != value['code']:
            codes.append(value)

    def on_ghost(msg):
        if not msg.hide and msg.state.joint_state.name:
            js = msg.state.joint_state
            ghost[:] = [js.position[js.name.index(name)] for name in JOINTS]

    node.create_subscription(String, '/piper_servo/guard/status', on_guard, qos)
    node.create_subscription(JointState, '/piper_servo/commanded_joint_states',
        lambda msg: q.__setitem__(slice(None), [msg.position[msg.name.index(name)] for name in JOINTS]), 1)
    node.create_subscription(ServoStatus, '/piper_servo/servo_node/status', on_status, 20)
    node.create_subscription(DisplayRobotState, '/piper_servo/target_robot_state', on_ghost, qos)
    heartbeat = node.create_publisher(Float64, '/piper_servo/panel/heartbeat', 1)
    marker = node.create_publisher(Feedback, '/piper_servo/target/feedback', 10)
    dragging = False
    target = None
    last_event = 0.
    calls = []
    process, stream = None, None
    owned = {}
    report = dict(scenario=scenario, simulation_only=True, sdk_or_can_started=False, passed=False)

    def event(kind, pose):
        nonlocal last_event
        msg = Feedback()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.marker_name, msg.client_id, msg.control_name = 'tcp_target', 'rotation_translation_test', 'move_free'
        msg.event_type, msg.pose = kind, pose_message(pose)
        marker.publish(msg)
        last_event = time.monotonic()

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            heartbeat.publish(Float64(data=time.time()))
            if dragging and target is not None and time.monotonic()-last_event > .20:
                event(Feedback.KEEP_ALIVE, target)
            rclpy.spin_once(node, timeout_sec=.01)
            if status and (not status_trace or status_trace[-1]['state'] != status.get('state')):
                status_trace.append(dict(at=time.monotonic(), **status))

    def wait(predicate, timeout=12., label='condition'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            spin(.02)
            if predicate():
                return
            if process is not None and process.poll() is not None:
                raise AssertionError(f'owned sim exited {process.returncode}')
        raise AssertionError(f'{label} timeout: status={status}; guard={guard}; q={q}')

    def call(path, kind, request):
        client = node.create_client(kind, '/piper_servo/' + path)
        try:
            wait(client.service_is_ready, label=path + ' ready')
            future = client.call_async(request)
            wait(future.done, label=path + ' response')
            calls.append(path)
            return future.result()
        finally:
            node.destroy_client(client)

    def command(name):
        response = call(name, Trigger, Trigger.Request())
        assert response.success, (name, response.message)
        return response

    def edit(pose, release=True):
        nonlocal dragging, target
        target = np.asarray(pose).copy()
        old_revision = status['target_revision']
        event(Feedback.MOUSE_DOWN, model.pose(q)); dragging = True
        spin(.05)
        event(Feedback.POSE_UPDATE, target)
        if release:
            spin(.05); event(Feedback.MOUSE_UP, target); dragging = False
        wait(lambda: status.get('target_revision', -1) > old_revision and
             status.get('reachability') == '可达', label='valid target preview')

    try:
        spin(.8)
        others = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
                  if name != 'rotation_translation_sim_test']
        assert not others, f'Domain91 not empty: {others}'
        with tempfile.TemporaryDirectory(prefix='servo_rotation_translation_') as temporary:
            temporary = Path(temporary)
            config = load_config(ROOT / 'scripts/task/servo_tracking/config/config.yaml')
            config.update(initial_joints=list(map(float, initial_q)), workspace_samples=4096, workspace_resolution=.06)
            path = temporary / 'config.yaml'
            path.write_text(yaml.safe_dump(config))
            stream = (OUTPUT / f'rotation_translation_{scenario}.launcher.log').open('w')
            process = subprocess.Popen([sys.executable, str(ROOT / 'scripts/task/servo_tracking/run.py'),
                '--mode', 'sim', '--no-rviz', '--config', str(path)], stdout=stream,
                stderr=subprocess.STDOUT, start_new_session=True)
            wait(lambda: q and status.get('mode') == guard.get('mode') == 'sim' and
                 status.get('target_revision', 0) > 0 and
                 status.get('ghost_revision') == status.get('target_revision'), 40., 'sim marker startup')
            assert not any('agx_arm_ctrl' in name for name, _ in node.get_node_names_and_namespaces())
            response = call('move_group/get_parameters', GetParameters, GetParameters.Request(names=['robot_description']))
            model = RobotModel(response.values[0].string_value)
            assert status.get('control_algorithm') == config['control']['algorithm']
            report['effective_control'] = dict(algorithm=config['control']['algorithm'],
                linear_speed=config['linear_speed'], angular_speed=config['angular_speed'],
                joint_speed=config['joint_speed'], control_source=config['control_config'])
            controller_deadline = time.monotonic()+20.
            while True:
                controllers = call('controller_manager/list_controllers', ListControllers, ListControllers.Request())
                if any(controller.name == 'arm_controller' and controller.state == 'active'
                       for controller in controllers.controller):
                    break
                assert time.monotonic() < controller_deadline, 'arm_controller activation timeout'
                spin(.1)
            wait(lambda: node.count_subscribers('/piper_servo/arm_controller/joint_trajectory') == 1,
                 label='JTC trajectory input discovery')
            # InteractiveMarkerServer initially owns the marker with an empty
            # client ID and rejects a different client during its 1s lease.
            # An ordinary RViz drag emits repeated POSE_UPDATE; this test sends
            # one fixed goal, so wait for that initial lease to expire first.
            spin(1.1)
            report['initial_joints'] = q[:]
            np.testing.assert_allclose(q, initial_q, atol=1e-9)
            if scenario in ('boundary', 'retarget'):
                target = np.asarray(case['target_pose7'])
                if options.retarget_recovery:
                    # The old millimetric fixture can now settle inside P's
                    # deadband before reaching the soft bound. Keep its XYZ,
                    # but request a further wrist-axis rotation to exercise an
                    # actual boundary/recovery with the current controller.
                    axis = model.jacobian(initial_q)[3:, 3]
                    target[3:] = (Rotation.from_rotvec(.20*axis) *
                                  Rotation.from_quat(target[3:])).as_quat()
                    report['same_xyz_wrist_rotation_rad'] = .20
                # A different wrist branch can be valid even when the local
                # differential Servo direction runs into J4's soft upper bound.
                candidate = np.asarray(case['candidate_ik_joints'])
                request = GetPositionIK.Request()
                request.ik_request.group_name, request.ik_request.ik_link_name = 'arm', 'tcp_link'
                request.ik_request.avoid_collisions = True
                request.ik_request.robot_state.joint_state.name = JOINTS
                request.ik_request.robot_state.joint_state.position = candidate.tolist()
                request.ik_request.pose_stamped.header.frame_id = 'base_link'
                request.ik_request.pose_stamped.pose = pose_message(target)
                request.ik_request.timeout.nanosec = 30_000_000
                response = call('compute_ik', GetPositionIK, request)
                assert response.error_code.val == response.error_code.SUCCESS, response.error_code.val
                js = response.solution.joint_state
                exact_goal = np.array([js.position[js.name.index(name)] for name in JOINTS])
                model.check_joints(exact_goal, config['joint_margin'])
                dp, dr = pose_error(target, model.pose(exact_goal))
                assert np.linalg.norm(dp) < .003 and np.linalg.norm(dr) < .03
                report['target_ik'] = dict(collision_checked=True, joints=exact_goal.tolist(),
                    position_error_mm=float(np.linalg.norm(dp)*1000), orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))))
                edit(target, release=False)
                revision = status['target_revision']
                command('start')
                # start clears drag capture, so capture again without changing the goal.
                event(Feedback.MOUSE_DOWN, target); dragging = True
                wait(lambda: any(item['code'] == 6 for item in codes), 12., 'native joint-bound code6')
                wait(lambda: any(state == 'paused' for state, _, _ in transitions), label='native pause')
                event(Feedback.MOUSE_UP, target); dragging = False
                report['native_joint_bound'] = next(item for item in codes if item['code'] == 6)
                assert status['active'], status
                if options.baseline:
                    spin(.4)
                    assert guard['state'] == 'paused' and status['active']
                    report['baseline_paused_same_goal'] = True
                    command('stop')
                    wait(lambda: guard.get('can_plan'), label='stop confirmed')
                    response = call('guard/plan_joint_target', PlanJointTarget, PlanJointTarget.Request(
                        joints=exact_goal.tolist(), speed_scale=.5, purpose='point',
                        target_revision=revision, auto_execute=False))
                    assert response.success, response.message
                    plan_id = response.plan_id
                    wait(lambda: guard.get('plan_id') == plan_id and guard.get('plan_ready'), 40., 'official joint plan')
                    response = call('guard/joint_plan_command', JointPlanCommand,
                                    JointPlanCommand.Request(plan_id=plan_id, command='execute'))
                    assert response.success, response.message
                    wait(lambda: guard.get('state') == 'joint_done', 40., 'official joint execution')
                    report['same_goal_official_joint_plan_executed'] = True
                else:
                    wait(lambda: any(state.startswith('joint_') for state, _, _ in transitions),
                         20., 'automatic joint recovery')
                    if options.retarget_recovery:
                        replaced = []
                        wait(lambda: guard.get('state') == 'joint_executing', 40., 'running recovery action')
                        for index in range(3):
                            previous_id = guard['plan_id']
                            spin(.25)
                            # Keep XYZ fixed while editing orientation during
                            # the wrist-branch action, as in the reported drag.
                            target = target.copy()
                            target[3:] = (Rotation.from_rotvec([.02, 0., 0.]) *
                                          Rotation.from_quat(target[3:])).as_quat()
                            edit(target)
                            revision = status['target_revision']
                            wait(lambda: guard.get('state') == 'joint_executing' and
                                 guard.get('plan_id') != previous_id, 40., 'replacement recovery action')
                            assert not foreign_actions, foreign_actions
                            replaced.append(dict(old_plan_id=previous_id, new_plan_id=guard['plan_id'],
                                                 revision=revision))
                        report['replaced_recovery_actions'] = replaced
                    wait(lambda: guard.get('state') in ('holding', 'joint_done') and
                         status.get('active') and not status.get('busy') and
                         '保持目标' in status.get('state', '') and guard.get('gate_closed'),
                         50., 'same-goal recovery and closed hold')
                    assert status['target_revision'] == revision
                    assert calls.count('start') == 1 and 'reset' not in calls and 'control_mode' not in calls
                    report.update(automatic_joint_recovery=True, same_revision_preserved=True,
                                  session_preserved=True, no_new_start_or_reset=True)
                dp, dr = pose_error(target, model.pose(q))
                assert np.linalg.norm(dp) < .0031 and np.linalg.norm(dr) < .031
                report.update(final_position_error_mm=float(np.linalg.norm(dp)*1000),
                              final_orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))), final_joints=q[:])
                if options.retarget_recovery:
                    target = target.copy()
                    target[1] += .005
                    edit(target)
                    wait(lambda: guard.get('state') == 'holding' and status.get('active') and
                         status.get('position_error_mm', 1000.) < 3.1 and
                         status.get('orientation_error_deg', 1000.) < 1.8,
                         20., 'Servo resumes after canceled recovery goals')
                    command('stop')
                    wait(lambda: guard.get('can_plan') and not guard.get('pending_joint_actions'),
                         label='all own actions terminal')
                    command('start')
                    wait(lambda: status.get('active') and guard.get('state') in ('running', 'holding'),
                         label='explicit Servo restart')
                    assert not foreign_actions, foreign_actions
                    report.update(servo_after_retarget=True, explicit_stop_start=True,
                                  own_goals_never_classified_as_foreign=True)
            else:
                command('start')
                wait(lambda: guard.get('state') == 'running', label='normal servo running')
                operations = []
                for operation in ('translate', 'rotate', 'rotate', 'translate'):
                    pose = model.pose(q)
                    if operation == 'translate':
                        pose[1] += .006
                    else:
                        pose[3:] = (Rotation.from_rotvec([0., .04, .02]) * Rotation.from_quat(pose[3:])).as_quat()
                    edit(pose)
                    def reached_this_goal():
                        dp, dr = pose_error(pose, model.pose(q))
                        return (guard.get('state') == 'holding' and status.get('active') and
                                np.linalg.norm(dp) <= config['hold_position_tolerance'] + 1e-5 and
                                np.linalg.norm(dr) <= config['hold_orientation_tolerance'] + 1e-5)
                    wait(reached_this_goal, 15., operation + ' hold')
                    dp, dr = pose_error(pose, model.pose(q))
                    assert np.linalg.norm(dp) <= config['hold_position_tolerance'] + 1e-5
                    assert np.linalg.norm(dr) <= config['hold_orientation_tolerance'] + 1e-5
                    operations.append(dict(operation=operation, position_error_mm=float(np.linalg.norm(dp)*1000),
                                           orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr)))))
                assert not any(item['code'] == 6 for item in codes)
                assert not any(state.startswith('joint_') for state, _, _ in transitions)
                assert calls.count('start') == 1 and status['active']
                report.update(operations=operations, ordinary_motion_no_fallback=True, both_orders_keep_session=True)
            report['passed'] = True
    except Exception as exc:
        report.update(error=repr(exc), traceback=traceback.format_exc())
        print(report['traceback'], file=sys.stderr, flush=True)
    finally:
        dragging = False
        if process is not None:
            owned = process_records(process.pid)
            try:
                if process.poll() is None:
                    command('stop')
            except Exception:
                pass
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            deadline = time.monotonic()+15.
            while process.poll() is None and time.monotonic() < deadline:
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
            except subprocess.TimeoutExpired:
                report['passed'] = False
        if stream is not None:
            stream.close()
        remaining = [pid for pid, started in owned.items() if same_process(pid, started)]
        report.update(guard_transitions=transitions, servo_codes=codes, status_trace=status_trace,
                      foreign_actions=foreign_actions,
                      service_calls=calls, last_status=status, last_guard=guard,
                      cleanup=dict(remaining=remaining, owned_pids=list(owned)))
        if remaining:
            report['passed'] = False
        node.destroy_node()
        rclpy.shutdown()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', action='store_true')
    parser.add_argument('--expect-auto', action='store_true')
    parser.add_argument('--edge-only', action='store_true')
    parser.add_argument('--retarget-recovery', action='store_true')
    parser.add_argument('--case', type=Path, default=DEFAULT_CASE)
    parser.add_argument('--candidate-ik', type=Path, default=DEFAULT_IK)
    options = parser.parse_args()
    if options.baseline == options.expect_auto:
        parser.error('Choose exactly one of --baseline / --expect-auto')
    if options.retarget_recovery and not options.expect_auto:
        parser.error('--retarget-recovery requires --expect-auto')
    assert os.environ.get('ROS_DOMAIN_ID') == '91', 'Use dedicated PIPERL_ROS_DOMAIN_ID=91 runtime wrapper'
    lock = open('/tmp/servo_rotation_translation_domain91.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    case = json.loads(options.case.read_text())
    candidate = json.loads(options.candidate_ik.read_text())
    # The offline report schema is deliberately read explicitly; no generated
    # IK branch is a command until the official IK and joint planner validate it.
    solutions = candidate.get('solutions', [])
    if not solutions or 'q' not in solutions[0]:
        parser.error('candidate-ik report must provide a solutions[0].q branch')
    case['candidate_ik_joints'] = solutions[0]['q']
    report = dict(simulation_only=True, domain=91, baseline=options.baseline,
                  source_case=str(options.case), source_scope=case['scope'], passed=False)
    report['boundary'] = run_stack(case['raw_joints'], case, options,
                                   'retarget' if options.retarget_recovery else 'boundary')
    if not options.edge_only and report['boundary']['passed']:
        report['ordinary'] = run_stack([0., 1.4, -1.2, .3, 1., 0.], case, options, 'ordinary')
    report['passed'] = report['boundary']['passed'] and (options.edge_only or report.get('ordinary', {}).get('passed', False))
    suffix = 'baseline' if options.baseline else 'auto'
    if options.retarget_recovery:
        suffix += '_retarget'
    path = OUTPUT / f'rotation_translation_{suffix}.json'
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    lock.close()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
