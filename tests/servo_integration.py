"""Opt-in, simulation-only integration test. Run against an isolated --mode sim stack."""
import json
from pathlib import Path
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback, MarkerArray, Marker, InteractiveMarkerControl
from visualization_msgs.srv import GetInteractiveMarkers
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetPositionFK
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.task.servo_tracking.core import RobotModel, JOINTS, pose_error
from scripts.task.servo_tracking.node import pose_message


def main():
    rclpy.init()
    n=rclpy.create_node('servo_sim_integration')
    status={};guard={};joints=[];workspace=[];twist_peaks=[0.,0.]
    n.create_subscription(String,'/piper_servo/status',lambda m:status.update(json.loads(m.data)),1)
    n.create_subscription(String,'/piper_servo/guard/status',lambda m:guard.update(json.loads(m.data)),1)
    def on_joints(m):
        joints[:]=[m.position[m.name.index(name)] for name in JOINTS]
    n.create_subscription(JointState,'/piper_servo/commanded_joint_states',on_joints,1)
    n.create_subscription(MarkerArray,'/piper_servo/workspace',lambda m:workspace.extend(m.markers),
                          QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    heartbeat=n.create_publisher(Float64,'/piper_servo/panel/heartbeat',1)
    speed=n.create_publisher(Float64,'/piper_servo/speed_scale',1)
    def on_twist(msg):
        twist_peaks[0]=max(twist_peaks[0],np.linalg.norm([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z]))
        twist_peaks[1]=max(twist_peaks[1],np.linalg.norm([msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z]))
    n.create_subscription(TwistStamped,'/piper_servo/servo_node/delta_twist_cmds',on_twist,10)
    marker=n.create_publisher(Feedback,'/piper_servo/target/feedback',1)
    trajectory_count=[0]
    n.create_subscription(JointTrajectory,'/piper_servo/servo/trajectory',
                          lambda msg:trajectory_count.__setitem__(0,trajectory_count[0]+1),1)
    send_heartbeat=True
    dragging=False
    target=None
    last_event=0.
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if send_heartbeat:heartbeat.publish(Float64(data=time.time()))
            if dragging and time.monotonic()-last_event>.25:event(Feedback.KEEP_ALIVE,target)
            rclpy.spin_once(n,timeout_sec=.025)
    def wait(predicate,timeout=6):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            spin(.05)
            if predicate():return
        raise AssertionError(f'timeout: status={status}, guard={guard}')
    def call(service,typ,request):
        c=n.create_client(typ,service)
        assert c.wait_for_service(timeout_sec=5),service
        f=c.call_async(request)
        wait(f.done)
        result=f.result();n.destroy_client(c)
        return result
    def event(kind,pose):
        nonlocal last_event
        msg=Feedback();msg.header.frame_id='base_link';msg.header.stamp=n.get_clock().now().to_msg()
        msg.client_id='simulation_test';msg.marker_name='tcp_target';msg.control_name='move_x'
        msg.event_type=kind;msg.pose=pose_message(pose);marker.publish(msg)
        last_event=time.monotonic()
    def command(name):
        result=call('/piper_servo/'+name,Trigger,Trigger.Request())
        assert result.success,result.message
    wait(lambda:bool(status) and bool(guard) and bool(joints) and bool(workspace))
    assert status['mode']=='sim' and guard['mode']=='sim','NEVER run this test against hardware'
    assert not any(name=='agx_arm_ctrl_single_node' and ns=='/piper_servo' for name,ns in n.get_node_names_and_namespaces())
    result=call('/piper_servo/move_group/get_parameters',GetParameters,GetParameters.Request(names=['robot_description']))
    model=RobotModel(result.values[0].string_value)
    assert workspace[0].color.g>workspace[0].color.r and 0<workspace[0].color.a<.3
    assert workspace[0].type==Marker.TRIANGLE_LIST
    assert 1000<len(workspace[0].points)<=60000 and len(workspace[0].points)%3==0
    controls=call('/piper_servo/target/get_interactive_markers',GetInteractiveMarkers,GetInteractiveMarkers.Request()).markers[0].controls
    assert controls[0].name=='move_free' and controls[0].interaction_mode==InteractiveMarkerControl.MOVE_PLANE
    assert controls[0].orientation_mode==InteractiveMarkerControl.VIEW_FACING
    reports=[]
    hold_position=status['hold_position_tolerance_mm']/1000
    hold_orientation=np.deg2rad(status['hold_orientation_tolerance_deg'])
    # Independently compare our FK to MoveIt's model before trusting Cartesian error measurements.
    fk=GetPositionFK.Request();fk.header.frame_id='base_link';fk.fk_link_names=['tcp_link']
    fk.robot_state.joint_state.name=JOINTS;fk.robot_state.joint_state.position=joints[:]
    result=call('/piper_servo/compute_fk',GetPositionFK,fk)
    from scripts.task.servo_tracking.node import pose_values
    dp,dr=pose_error(model.pose(joints),pose_values(result.pose_stamped[0].pose))
    assert np.linalg.norm(dp)<1e-7 and np.linalg.norm(dr)<1e-7
    command('start')
    wait(lambda:guard.get('state')=='running' and status.get('reachability') in ('可达','当前位置'))
    for axis in range(6):
        initial=model.pose(joints)
        target=initial.copy()
        event(Feedback.MOUSE_DOWN,initial);dragging=True;spin(.1)
        if axis<3:target[axis]+=.004
        else:
            dr=np.zeros(3);dr[axis-3]=.035
            target[3:]=(Rotation.from_rotvec(dr)*Rotation.from_quat(initial[3:])).as_quat()
        event(Feedback.POSE_UPDATE,target)
        spin(.1)
        event(Feedback.MOUSE_UP,target);dragging=False
        # Release while still in transit. One start must cover all six separate drags.
        spin(2.5)
        assert status['active'],status
        dp,dr=pose_error(target,model.pose(joints))
        assert np.linalg.norm(dp)<=hold_position+1e-5,(axis,dp,status)
        assert np.linalg.norm(dr)<=hold_orientation+1e-5,(axis,dr,status)
        reports.append({'axis':axis,'position_error_mm':float(np.linalg.norm(dp)*1000),
                        'orientation_error_deg':float(np.rad2deg(np.linalg.norm(dr)))})
        assert guard.get('state') in ('running','holding'),guard
    wait(lambda:guard.get('state')=='holding')
    spin(.2)
    held=np.asarray(joints).copy();count=trajectory_count[0];spin(.8)
    assert status['active'] and guard['state']=='holding'
    assert np.max(np.abs(np.asarray(joints)-held))<1e-7
    assert trajectory_count[0]==count,'settled Servo must pause, not stream correction trajectories'
    # Continuously moving six-axis target, similar to real mouse feedback at 30 Hz.
    initial=model.pose(joints);target=initial.copy()
    event(Feedback.MOUSE_DOWN,initial);dragging=True;spin(.1)
    lag=[]
    for i in range(90):
        fraction=(i+1)/90
        target[:3]=initial[:3]+fraction*np.array([.015,.005,.005])
        target[3:]=(Rotation.from_rotvec(fraction*np.array([.03,.04,.03]))*Rotation.from_quat(initial[3:])).as_quat()
        event(Feedback.POSE_UPDATE,target);spin(1/30)
        assert status['active'],status
        lag.append(float(np.linalg.norm(pose_error(target,model.pose(joints))[0])))
    event(Feedback.MOUSE_UP,target);dragging=False
    spin(1.6)
    dp,dr=pose_error(target,model.pose(joints))
    assert np.linalg.norm(dp)<.004 and np.linalg.norm(dr)<.025,(dp,dr,status)
    assert max(lag)<.010,('Continuous target lag exceeded 10 mm',max(lag))
    continuous=dict(updates=90,nominal_hz=30,max_position_lag_mm=max(lag)*1000,
                    settled_position_error_mm=float(np.linalg.norm(dp)*1000),
                    settled_orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))))
    assert status['active'] and guard.get('state') in ('running','holding')
    # A reachable target >12 cm away must be pursued at the configured speed, not disarm.
    initial=model.pose(joints)
    far_q=np.asarray(joints).copy();far_q[1]+=.35
    model.check_joints(far_q,.015)
    target=model.pose(far_q)
    far_distance=float(np.linalg.norm(target[:3]-initial[:3]))
    assert far_distance>.12,far_distance
    speed.publish(Float64(data=1.));spin(.15)
    event(Feedback.MOUSE_DOWN,initial);dragging=True;spin(.1)
    event(Feedback.POSE_UPDATE,target);spin(.1)
    event(Feedback.MOUSE_UP,target);dragging=False
    started=time.monotonic()
    def reached_far():
        assert status['active'] and guard.get('state') not in ('idle','fault','paused'),(status,guard)
        dp,dr=pose_error(target,model.pose(joints))
        return np.linalg.norm(dp)<=hold_position and np.linalg.norm(dr)<=hold_orientation
    wait(reached_far,timeout=20)
    dp,dr=pose_error(target,model.pose(joints))
    far_target=dict(initial_distance_mm=far_distance*1000,elapsed_s=time.monotonic()-started,
                    final_position_error_mm=float(np.linalg.norm(dp)*1000),
                    final_orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))))
    assert twist_peaks[0]<=status['linear_speed_limit_mps']+1e-6 and twist_peaks[1]<=status['angular_speed_limit_radps']+1e-6,twist_peaks
    command('stop')
    wait(lambda:not status['active'] and guard.get('state')=='idle')
    spin(.25);stopped=np.asarray(joints).copy();spin(.4)
    assert np.max(np.abs(np.asarray(joints)-stopped))<.001
    # Invalid edits hold the arm without discarding the tracking session.
    before=np.asarray(joints).copy();command('reset');spin(.3)
    assert np.max(np.abs(np.asarray(joints)-before))<.001
    command('start');wait(lambda:guard.get('state')=='running')
    target=model.pose(joints);event(Feedback.MOUSE_DOWN,target);dragging=True;spin(.1)
    target[:3]=[3.,3.,3.];before=np.asarray(joints).copy();event(Feedback.POSE_UPDATE,target)
    event(Feedback.MOUSE_UP,target);dragging=False
    wait(lambda:status.get('active') and guard.get('state')=='holding' and not status.get('reachability') in ('可达','当前位置','检查中'))
    spin(.3)
    assert np.max(np.abs(np.asarray(joints)-before))<.001
    assert status['active'] and guard['state']=='holding','invalid goal must not revoke the session'
    # A new valid edit can resume, without a second start or a reset.
    target=model.pose(joints);target[1]+=.004
    event(Feedback.MOUSE_DOWN,model.pose(joints));dragging=True;spin(.05)
    event(Feedback.POSE_UPDATE,target);spin(.08);event(Feedback.MOUSE_UP,target);dragging=False
    wait(lambda:status.get('reachability')=='可达' and guard.get('state')=='running')
    wait(lambda:status.get('position_error_mm',100.)<=hold_position*1000)
    assert status['active']
    # A disconnected panel is a stop event, even if a drag keep-alive was last received.
    send_heartbeat=False
    wait(lambda:not status.get('active'),timeout=2)
    command('stop')
    report={'six_axis_tracking':reports,'workspace_triangles':len(workspace[0].points)//3,
        'continuous_drag':continuous,
        'far_target_tracking':far_target,'max_twist_linear_mps':twist_peaks[0],
        'max_twist_angular_radps':twist_peaks[1],
        'fk_matches_moveit':True,'sphere_draggable_control':True,'one_start_multiple_drags':True,
        'release_converges_and_stays_active':True,'explicit_stop_holds':True,'reset_no_motion':True,
        'settled_holding_pauses_trajectory_stream':True,'new_drag_resumes_without_start':True,
        'unreachable_rejected':True,'invalid_goal_holds_session':True,
        'new_valid_goal_resumes_without_start':True,'panel_disconnect_stops':True}
    output=Path(__file__).resolve().parents[1]/'outputs/servo_tracking/validation'
    output.mkdir(parents=True,exist_ok=True)
    (output/'integration.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
