"""Opt-in isolated simulation: singular startup and guarded single-point control.

Use a sim config with initial_joints=[0]*6. Never attaches to a real-arm stack.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
from visualization_msgs.srv import GetInteractiveMarkers
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.task.servo_tracking.core import JOINTS, RobotModel, pose_error
from scripts.task.servo_tracking.node import pose_message, pose_values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--handoff-only', action='store_true', help='Set a valid goal before startup and verify ready→Servo')
    args = parser.parse_args()
    rclpy.init()
    n = rclpy.create_node('servo_modes_sim_test')
    status, guard, joints = {}, {}, []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, '/piper_servo/status', lambda m: status.update(json.loads(m.data)), qos)
    n.create_subscription(String, '/piper_servo/guard/status', lambda m: guard.update(json.loads(m.data)), qos)
    n.create_subscription(JointState, '/piper_servo/commanded_joint_states',
        lambda m: joints.__setitem__(slice(None), [m.position[m.name.index(k)] for k in JOINTS]), 1)
    heartbeat = n.create_publisher(Float64, '/piper_servo/panel/heartbeat', 1)
    marker = n.create_publisher(Feedback, '/piper_servo/target/feedback', 10)
    speed = n.create_publisher(Float64, '/piper_servo/speed_scale', 1)
    heartbeat_on = True

    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            if heartbeat_on:
                heartbeat.publish(Float64(data=time.time()))
            rclpy.spin_once(n, timeout_sec=.015)

    def wait(predicate, timeout=12.):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            spin(.04)
            if predicate():
                return
        raise AssertionError(dict(status=status, guard=guard, joints=joints))

    def call(path, typ, req):
        client = n.create_client(typ, '/piper_servo/'+path)
        assert client.wait_for_service(timeout_sec=4.), path
        future = client.call_async(req)
        wait(future.done)
        result = future.result()
        n.destroy_client(client)
        return result

    def command(name):
        result = call(name, Trigger, Trigger.Request())
        assert result.success, result.message
        return result

    def mode(point):
        result = call('control_mode', SetBool, SetBool.Request(data=point))
        assert result.success, result.message
        wait(lambda: status['control_mode'] == ('point' if point else 'servo'))

    def plan():
        old_id = status.get('plan_id')
        command('plan')
        wait(lambda: status['plan_ready'] and status.get('plan_id') != old_id)

    def marker_pose():
        result = call('target/get_interactive_markers', GetInteractiveMarkers, GetInteractiveMarkers.Request())
        return pose_values(result.markers[0].pose)

    def event(kind, pose):
        msg = Feedback()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.marker_name, msg.client_id, msg.control_name = 'tcp_target', 'modes_sim_test', 'move_free'
        msg.event_type, msg.pose = kind, pose_message(pose)
        marker.publish(msg)

    def edit(pose):
        revision = status['target_revision']
        event(Feedback.MOUSE_DOWN, marker_pose()); spin(.06)
        event(Feedback.POSE_UPDATE, pose); spin(.08)
        event(Feedback.MOUSE_UP, pose)
        wait(lambda: status['target_revision'] > revision and status['reachability'] == '可达')

    report = {'simulation_only': True}
    wait(lambda: bool(joints) and bool(status) and bool(guard))
    assert status['mode'] == guard['mode'] == 'sim', 'NEVER test against hardware'
    assert not any(name == 'agx_arm_ctrl_single_node' and ns == '/piper_servo'
                   for name, ns in n.get_node_names_and_namespaces())
    result = call('move_group/get_parameters', GetParameters, GetParameters.Request(names=['robot_description']))
    model = RobotModel(result.values[0].string_value)
    assert np.max(np.abs(joints)) < 1e-8, 'This test requires an isolated zero-initialized sim config'
    try:
        zero_pose = marker_pose(); revision = status['target_revision']
        spin(.3)
        assert np.max(np.abs(joints)) < 1e-8, 'launch must not cause startup motion'
        if args.handoff_only:
            zero_pose = model.pose([.08, 1.35, -1.15, .30, .95, .04])
            edit(zero_pose)
            revision = status['target_revision']
        command('start')
        wait(lambda: status['busy'])
        if args.handoff_only:
            wait(lambda: status['active'] and not status['busy'] and guard['state'] == 'running', 35.)
            wait(lambda: status['position_error_mm'] < 2. and status['orientation_error_deg'] < 1.2, 10.)
            dp, dr = pose_error(zero_pose, marker_pose())
            assert np.linalg.norm(dp) < 1e-8 and np.linalg.norm(dr) < 1e-8
            assert status['target_revision'] == revision
            report.update(zero_ready_to_servo_handoff=True, edited_goal_preserved=True,
                          position_error_mm=status['position_error_mm'], orientation_error_deg=status['orientation_error_deg'])
            output = Path(__file__).resolve().parents[1]/'outputs/servo_tracking/validation/modes_handoff.json'
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
            print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
            return
        wait(lambda: not status['active'] and not status['busy'] and '准备位已到达' in status['state'], 35.)
        assert model.condition_number(joints) < 17., guard
        dp, dr = pose_error(zero_pose, marker_pose())
        assert np.linalg.norm(dp) < 1e-8 and np.linalg.norm(dr) < 1e-8
        assert status['target_revision'] == revision, 'preparation must retain original goal'
        report.update(zero_start_joint_preparation=True, prepared_condition=model.condition_number(joints),
                      singular_target_preserved_without_returning=True, no_launch_motion=True)

        command('reset')
        mode(True)
        goal = model.pose([.10, 1.25, -1.10, .30, .85, .10])
        edit(goal)
        before = np.asarray(joints).copy()
        plan()
        assert np.max(np.abs(np.asarray(joints)-before)) < 1e-6, 'planning must not move'
        command('execute')
        wait(lambda: not status['busy'] and status['state'] == '单点已到达', 25.)
        dp, dr = pose_error(goal, model.pose(joints))
        assert np.linalg.norm(dp) < .003 and np.linalg.norm(dr) < .03
        report.update(point_plan_without_motion=True, point_execute_reached=True,
                      point_position_error_mm=float(np.linalg.norm(dp)*1000),
                      point_orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))))

        goal2 = model.pose([-.10, 1.40, -1.20, .40, 1., -.10])
        edit(goal2); plan()
        speed.publish(Float64(data=.6))
        wait(lambda: not status['plan_ready'] and '速度' in status['state'])
        assert not call('execute', Trigger, Trigger.Request()).success
        plan()
        command('execute'); wait(lambda: guard['state'] == 'joint_executing')
        mode(False)
        wait(lambda: guard['state'] == 'idle' and not status['busy'])
        held = np.asarray(joints).copy(); spin(.5)
        assert np.max(np.abs(np.asarray(joints)-held)) < .001
        assert not call('execute', Trigger, Trigger.Request()).success
        report.update(speed_change_invalidates_plan=True, mode_switch_cancels_execution=True,
                      old_plan_cannot_execute=True)

        command('reset')
        command('start'); wait(lambda: guard['state'] == 'running')
        target = model.pose(joints); target[1] += .005
        edit(target)
        wait(lambda: status['position_error_mm'] < 2.)
        assert status['active'], status
        command('stop'); mode(True)
        goal3 = model.pose([.05, 1.35, -1.15, .30, .95, .05])
        edit(goal3); plan()
        heartbeat_on = False
        wait(lambda: not status['plan_ready'] and not status['busy'] and '连接中断' in status['state'])
        heartbeat_on = True; spin(.2)
        assert not call('execute', Trigger, Trigger.Request()).success
        report.update(servo_after_point=True, panel_disconnect_invalidates_plan=True)
        output = Path(__file__).resolve().parents[1]/'outputs/servo_tracking/validation/modes.json'
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    finally:
        heartbeat_on = True
        command('stop')
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
