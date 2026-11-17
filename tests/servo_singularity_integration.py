"""Opt-in isolated simulation: inject Servo halt status, exercise safe recovery.

Never run in an operator session. No real driver is started or accepted by this test.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ServoStatus
from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
from rcl_interfaces.srv import GetParameters

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.task.servo_tracking.core import JOINTS, RobotModel, pose_error
from scripts.task.servo_tracking.node import pose_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--expect-singular-start', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('servo_singularity_sim_test')
    status, guard, joints, transitions = {}, {}, [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/piper_servo/status', lambda m: status.update(json.loads(m.data)), qos)
    def on_guard(msg):
        data = json.loads(msg.data)
        if data['state'] != guard.get('state'):
            transitions.append(data['state'])
        guard.update(data)
    node.create_subscription(String, '/piper_servo/guard/status', on_guard, qos)
    def on_joints(msg):
        joints[:] = [msg.position[msg.name.index(name)] for name in JOINTS]
    node.create_subscription(JointState, '/piper_servo/commanded_joint_states', on_joints, 1)
    heartbeat = node.create_publisher(Float64, '/piper_servo/panel/heartbeat', 1)
    feedback = node.create_publisher(Feedback, '/piper_servo/target/feedback', 10)
    injected = node.create_publisher(ServoStatus, '/piper_servo/servo_node/status', 1)
    send_heartbeat = True
    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            if send_heartbeat:
                heartbeat.publish(Float64(data=time.time()))
            rclpy.spin_once(node, timeout_sec=.02)
    def wait(predicate, timeout=6.):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            spin(.04)
            if predicate():
                return
        raise AssertionError((status, guard, transitions))
    def call(name, typ, request):
        client = node.create_client(typ, '/piper_servo/'+name)
        assert client.wait_for_service(timeout_sec=3.), name
        future = client.call_async(request)
        wait(future.done)
        result = future.result()
        node.destroy_client(client)
        return result
    def event(kind, pose):
        msg = Feedback()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.client_id, msg.marker_name, msg.control_name = 'isolated_singularity_test', 'tcp_target', 'move_free'
        msg.event_type, msg.pose = kind, pose_message(pose)
        feedback.publish(msg)
    def inject_halt():
        injected.publish(ServoStatus(code=2, message='simulation-only injected singularity halt'))
        wait(lambda: guard.get('state') == 'paused')
        wait(lambda: status.get('guard_state') == 'paused')
        assert status['active'], status
    wait(lambda: status and guard and joints)
    assert status['mode'] == guard['mode'] == 'sim', 'Simulation only; never test on hardware'
    assert not any(name == 'agx_arm_ctrl_single_node' and ns == '/piper_servo'
                   for name, ns in node.get_node_names_and_namespaces())
    result = call('move_group/get_parameters', GetParameters,
                  GetParameters.Request(names=['robot_description']))
    model = RobotModel(result.values[0].string_value)
    report = {'simulation_only': True, 'initial_condition_number': model.condition_number(joints)}
    try:
        result = call('start', Trigger, Trigger.Request())
        if args.expect_singular_start:
            assert model.condition_number(joints) >= 30.
            assert not result.success and '奇异' in result.message, result
            before = np.asarray(joints).copy()
            spin(.6)
            assert not status['active'] and guard['state'] == 'idle'
            assert np.max(np.abs(np.asarray(joints)-before)) < 1e-6
            report.update(singular_start_rejected=True, start_message=result.message, no_motion=True)
        else:
            assert result.success, result.message
            wait(lambda: guard.get('state') == 'running' and status.get('reachability') in ('可达', '当前位置'))
            initial = model.pose(joints)
            target = initial.copy(); target[0] += .010
            event(Feedback.MOUSE_DOWN, initial); spin(.08)
            event(Feedback.POSE_UPDATE, target); spin(.12)
            event(Feedback.MOUSE_UP, target); spin(.08)
            inject_halt()
            spin(.25)
            held = np.asarray(joints).copy()
            spin(.6)
            assert guard['state'] == 'paused' and status['active']
            assert np.max(np.abs(np.asarray(joints)-held)) < .001
            assert status.get('reachability') == '等待新目标', status
            # The old goal and ordinary zero-command status cannot authorize recovery.
            report.update(pause_preserves_session=True, old_goal_does_not_resume=True, paused_holds=True)
            actual = model.pose(joints)
            target = actual.copy(); target[1] += .004
            event(Feedback.MOUSE_DOWN, actual); spin(.08)
            event(Feedback.POSE_UPDATE, target); spin(.08)
            event(Feedback.MOUSE_UP, target)
            wait(lambda: guard.get('state') == 'running')
            spin(2.)
            dp, dr = pose_error(target, model.pose(joints))
            assert status['active'] and np.linalg.norm(dp) < .0025
            assert 'syncing' in transitions and transitions.count('running') >= 2, transitions
            report.update(fresh_target_resynchronizes=True, only_one_start_call=True,
                          resumed_error_mm=float(np.linalg.norm(dp)*1000))
            inject_halt()
            send_heartbeat = False
            wait(lambda: not status.get('active'), timeout=2.)
            report['panel_disconnect_during_pause_stops'] = True
        report['guard_transitions'] = transitions
        out = Path(__file__).resolve().parents[1]/'outputs/servo_tracking/validation'
        out.mkdir(parents=True, exist_ok=True)
        name = 'singular_start.json' if args.expect_singular_start else 'singularity_recovery.json'
        (out/name).write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    finally:
        call('stop', Trigger, Trigger.Request())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
