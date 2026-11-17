"""Opt-in isolated sim: target ghost, pose error, start-retains-goal and manual reset."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from moveit_msgs.msg import DisplayRobotState
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
from visualization_msgs.srv import GetInteractiveMarkers
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.task.servo_tracking.core import JOINTS, RobotModel, pose_error
from scripts.task.servo_tracking.node import pose_message, pose_values


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--preview-only',action='store_true')
    parser.add_argument('--expect-zero-hold',action='store_true')
    args=parser.parse_args()
    rclpy.init();n=rclpy.create_node('servo_goal_preview_sim_test')
    status={};joints=[];ghost=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String,'/piper_servo/status',lambda m:status.update(json.loads(m.data)),qos)
    n.create_subscription(DisplayRobotState,'/piper_servo/target_robot_state',lambda m:ghost.__setitem__(slice(None),[m]),qos)
    def on_joints(m):
        joints[:]=[m.position[m.name.index(name)] for name in JOINTS]
    n.create_subscription(JointState,'/piper_servo/commanded_joint_states',on_joints,1)
    heartbeat=n.create_publisher(Float64,'/piper_servo/panel/heartbeat',1)
    marker=n.create_publisher(Feedback,'/piper_servo/target/feedback',10)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            heartbeat.publish(Float64(data=time.time()));rclpy.spin_once(n,timeout_sec=.02)
    def wait(predicate,timeout=8.):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            spin(.04)
            if predicate():return
        raise AssertionError(status)
    def call(path,typ,request):
        client=n.create_client(typ,'/piper_servo/'+path)
        assert client.wait_for_service(timeout_sec=3),path
        future=client.call_async(request);wait(future.done)
        result=future.result();n.destroy_client(client);return result
    def command(name):return call(name,Trigger,Trigger.Request())
    def marker_pose():
        result=call('target/get_interactive_markers',GetInteractiveMarkers,GetInteractiveMarkers.Request())
        return pose_values(result.markers[0].pose)
    def event(kind,pose):
        msg=Feedback();msg.header.frame_id='base_link';msg.header.stamp=n.get_clock().now().to_msg()
        msg.client_id='goal_preview_test';msg.marker_name='tcp_target';msg.control_name='move_free'
        msg.event_type=kind;msg.pose=pose_message(pose);marker.publish(msg)
    def edit(target):
        event(Feedback.MOUSE_DOWN,marker_pose());spin(.06)
        event(Feedback.POSE_UPDATE,target);spin(.08)
        event(Feedback.MOUSE_UP,target)
    def ghost_q():
        msg=ghost[0].state.joint_state
        return np.array([msg.position[msg.name.index(name)] for name in JOINTS])
    wait(lambda:bool(status) and bool(joints) and bool(ghost))
    assert status['mode']=='sim' and not status['active']
    assert not any(name=='agx_arm_ctrl_single_node' and ns=='/piper_servo' for name,ns in n.get_node_names_and_namespaces())
    result=call('move_group/get_parameters',GetParameters,GetParameters.Request(names=['robot_description']))
    model=RobotModel(result.values[0].string_value)
    report={'simulation_only':True}
    try:
        if args.expect_zero_hold:
            spin(.6)
            assert np.max(np.abs(joints))<1e-8
            assert status['reachability']=='当前位置' and status['singularity_level']=='halt',status
            assert not ghost[0].hide
            assert np.max(np.abs(joints))<1e-8
            report.update(zero_pose_displayed_as_current_hold=True,no_false_margin_error=True,
                          no_automatic_launch_motion=True)
        else:
            initial=np.asarray(joints).copy();actual=model.pose(initial)
            target=actual.copy();target[:3]+=[0.,.03,.01]
            target[3:]=(Rotation.from_rotvec([0.,0.,.2])*Rotation.from_quat(actual[3:])).as_quat()
            edit(target)
            wait(lambda:status['reachability']=='可达' and status['ghost_revision']==status['target_revision'] and not ghost[0].hide)
            spin(.1)
            dp,dr=pose_error(target,model.pose(ghost_q()))
            assert np.linalg.norm(dp)<.003 and np.linalg.norm(dr)<.03
            assert np.max(np.abs(np.asarray(joints)-initial))<1e-6,'preview must not move actual robot'
            assert status['position_error_mm']>25 and status['orientation_error_deg']>10,status
            assert all(h.color.r>.9 and h.color.g>.8 and h.color.b<.1 for h in ghost[0].highlight_links)
            report.update(ghost_matches_target=True,preview_no_motion=True,
                          preview_position_error_mm=status['position_error_mm'],
                          preview_orientation_error_deg=status['orientation_error_deg'])
            if not args.preview_only:
                revision=status['target_revision'];assert command('start').success
                wait(lambda:status.get('guard_state')=='running')
                dp,dr=pose_error(target,marker_pose())
                assert np.linalg.norm(dp)<1e-6 and np.linalg.norm(dr)<1e-6
                assert status['target_revision']==revision,'start must not reset the goal'
                wait(lambda:status['position_error_mm']<2 and status['orientation_error_deg']<1.2,timeout=10.)
                assert command('stop').success;wait(lambda:not status['active'])
                report.update(start_preserves_pose_and_revision=True,existing_goal_followed=True)
                bad=target.copy();bad[:3]=[3.,3.,3.];edit(bad)
                wait(lambda:status['reachability']!='检查中' and ghost[0].hide)
                before=np.asarray(joints).copy()
                result=command('start')
                assert not result.success and '重置' in result.message
                dp,dr=pose_error(bad,marker_pose())
                assert np.linalg.norm(dp)<1e-6 and np.linalg.norm(dr)<1e-6
                spin(.2);assert np.max(np.abs(np.asarray(joints)-before))<.001
                assert command('reset').success
                wait(lambda:status['reachability']=='当前位置' and not ghost[0].hide)
                dp,dr=pose_error(model.pose(joints),marker_pose())
                assert np.linalg.norm(dp)<1e-5 and np.linalg.norm(dr)<1e-5
                report.update(invalid_goal_rejected_without_reset=True,invalid_ghost_hidden=True,manual_reset_aligns=True)
        output=Path(__file__).resolve().parents[1]/'outputs/servo_tracking/validation'
        output.mkdir(parents=True,exist_ok=True)
        name='zero_hold.json' if args.expect_zero_hold else 'ghost_preview.json' if args.preview_only else 'goal_preview.json'
        (output/name).write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    finally:
        command('stop');n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
