"""ROS message/state-machine regressions using fakes only; never starts hardware."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import time
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
try:
    from scripts.task.servo_tracking.guard import Guard
    from scripts.task.servo_tracking.node import Backend
    from scripts.task.servo_tracking.cartesian_control import CartesianController
    from scripts.task.servo_tracking.core import FreshJoints, RobotModel, JOINTS, load_config
    from sensor_msgs.msg import JointState
    from builtin_interfaces.msg import Time
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback, InteractiveMarkerControl as Control
    from scripts.task.servo_tracking.node import pose_message
    ROS=True
except ImportError:
    ROS=False


@unittest.skipUnless(ROS,'Use ROS runtime wrapper; all driver calls are mocked')
class ServoProtocolTests(unittest.TestCase):
    def test_late_gate_open_after_stop_recloses_gate(self):
        gate=Mock();future=Future();future.set_result(NS(success=True))
        state=NS(epoch=3,state='idle',real=True,closed=True,gate=gate)
        Guard.opened(state,future,2)
        self.assertEqual(state.state,'idle')
        self.assertFalse(state.closed)
        self.assertFalse(gate.call_async.call_args.args[0].data)

    def test_failed_gate_open_never_enters_running(self):
        future=Future();future.set_result(NS(success=False))
        state=NS(epoch=2,state='opening',stop=Mock())
        Guard.opened(state,future,2)
        state.stop.assert_called_once_with('控制门开启失败',fault=True)
        self.assertNotEqual(state.state,'running')

    def test_old_ik_result_cannot_authorize_new_target(self):
        future=Future();future.set_result(None)
        state=NS(pending=future,revision=7,valid=False,active=False)
        Backend.ik_done(state,future,6)
        self.assertFalse(state.valid)
        self.assertIsNone(state.pending)

    def test_cancelled_ik_does_not_restore_permission(self):
        future=Future();future.cancel()
        state=NS(pending=None,revision=7,valid=False)
        Backend.ik_done(state,future,7)
        self.assertFalse(state.valid)

    def test_controller_state_is_not_forwarded_during_sync(self):
        state=NS(commanded=FreshJoints(),real=True,state='syncing',driver_output=Mock())
        msg=JointState();msg.name=JOINTS;msg.position=[0.]*6;msg.header.stamp.sec=int(time.time())
        Guard.on_commanded(state,msg)
        state.driver_output.publish.assert_not_called()

    def test_stale_real_feedback_blocks_mock_command(self):
        state=NS(commanded=FreshJoints(),real=True,state='running',driver_output=Mock(),
                 health=Mock(side_effect=ValueError('stale real feedback')),stop=Mock())
        msg=JointState();msg.name=JOINTS;msg.position=[0.]*6;msg.header.stamp.sec=int(time.time())
        Guard.on_commanded(state,msg)
        state.driver_output.publish.assert_not_called()
        state.stop.assert_called_once_with('stale real feedback',fault=True)

    def test_other_action_cancels_and_revokes_control(self):
        state=NS(cancel_action=Mock(),state='running',stop=Mock())
        Guard.on_action_status(state,NS(status_list=[NS(status=2)]))
        self.assertTrue(state.action_active)
        state.cancel_action.call_async.assert_called_once()
        state.stop.assert_called_once_with('检测到其他轨迹任务',fault=True)

    def test_pending_target_tracks_only_last_verified_pose(self):
        config=load_config(Path(__file__).resolve().parents[1]/'scripts/task/servo_tracking/config/config.yaml')
        actual=np.array([0.,0.,0.,0.,0.,0.,1.])
        approved=actual.copy();approved[0]=.002
        unchecked=actual.copy();unchecked[0]=.004
        state=NS(heartbeat=Mock(),switched=True,actual=Mock(return_value=(np.zeros(6),actual)),
                 target=unchecked,approved_target=approved,active=True,dragging=True,
                 panel_seen=time.monotonic(),drag_seen=time.monotonic(),guard_state='running',
                 verification_gap_since=time.monotonic(),config=config,scale=.5,
                 publish_twist=Mock(),stop_tracking=Mock(),reject=Mock(),maybe_hold=Mock(),hold_future=None)
        state.controller=CartesianController(config['control'])
        state.approved_target_time=time.monotonic()
        Backend.tick(state)
        command=state.publish_twist.call_args.args[0]
        self.assertAlmostEqual(command[0],.002*config['position_gain'])  # Uses the approved pose only.
        state.stop_tracking.assert_not_called()
        state.verification_gap_since=time.monotonic()-1.
        Backend.tick(state)
        state.stop_tracking.assert_not_called()
        state.reject.assert_called_once_with('等待目标验证，请继续调整目标')
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0],np.zeros(6))

    def test_sphere_is_directly_draggable_without_modifier_keys(self):
        state=NS(server=Mock(),on_marker=Mock())
        Backend.marker(state,np.array([0.,0.,0.,0.,0.,0.,1.]))
        marker=state.server.insert.call_args.args[0]
        sphere=marker.controls[0]
        self.assertEqual(sphere.name,'move_free')
        self.assertEqual(sphere.interaction_mode,Control.MOVE_PLANE)
        self.assertEqual(sphere.orientation_mode,Control.VIEW_FACING)
        self.assertGreaterEqual(sphere.markers[0].scale.x,.035)
        self.assertEqual(len(marker.controls),7)

    def test_mouse_release_preserves_session_and_verifies_final_pose(self):
        state=NS(drag_client='rviz',active=True,dragging=True,update_target=Mock(),stop_tracking=Mock())
        msg=Feedback();msg.marker_name='tcp_target';msg.header.frame_id='base_link'
        msg.client_id='rviz';msg.event_type=Feedback.MOUSE_UP
        Backend.on_marker(state,msg)
        self.assertTrue(state.active)
        self.assertFalse(state.dragging)
        state.stop_tracking.assert_not_called()
        state.update_target.assert_called_once_with(msg.pose)

    def test_tracking_continues_after_release_but_stop_blocks_commands(self):
        config=load_config(Path(__file__).resolve().parents[1]/'scripts/task/servo_tracking/config/config.yaml')
        actual=np.array([0.,0.,0.,0.,0.,0.,1.]);target=actual.copy();target[0]=.004
        state=NS(heartbeat=Mock(),switched=True,actual=Mock(return_value=(np.zeros(6),actual)),
                 target=target,approved_target=target,active=True,dragging=False,
                 panel_seen=time.monotonic(),drag_seen=0.,guard_state='running',
                 verification_gap_since=None,config=config,scale=.5,
                 publish_twist=Mock(),stop_tracking=Mock(),maybe_hold=Mock(),hold_future=None)
        state.controller=CartesianController(config['control'])
        state.approved_target_time=time.monotonic()
        Backend.tick(state)
        self.assertGreater(state.publish_twist.call_args.args[0][0],0.)
        state.stop_tracking.assert_not_called()
        state.active=False
        Backend.tick(state)
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0],np.zeros(6))

    def test_repeated_release_pose_does_not_restart_validation(self):
        p=np.array([.1,0.,.4,0.,0.,0.,1.])
        state=NS(target=p,revision=9,valid=True,stop_tracking=Mock())
        Backend.update_target(state,pose_message(p))
        self.assertEqual(state.revision,9)
        self.assertTrue(state.valid)

    def test_far_position_and_orientation_targets_keep_speed_limits_without_disarming(self):
        config=load_config(Path(__file__).resolve().parents[1]/'scripts/task/servo_tracking/config/config.yaml')
        actual=np.array([0.,0.,0.,0.,0.,0.,1.])
        target=np.r_[[.25,.1,0.],Rotation.from_rotvec([0.,0.,1.2]).as_quat()]
        state=NS(heartbeat=Mock(),switched=True,actual=Mock(return_value=(np.zeros(6),actual)),
                 target=target,approved_target=target,active=True,dragging=False,
                 panel_seen=time.monotonic(),guard_state='running',verification_gap_since=None,
                 config=config,scale=.5,publish_twist=Mock(),stop_tracking=Mock(),maybe_hold=Mock(),hold_future=None)
        state.controller=CartesianController(config['control'])
        state.approved_target_time=time.monotonic()
        Backend.tick(state)
        command=state.publish_twist.call_args.args[0]
        self.assertAlmostEqual(np.linalg.norm(command[:3]),config['linear_speed']*.5)
        self.assertAlmostEqual(np.linalg.norm(command[3:]),config['angular_speed']*.5)
        state.stop_tracking.assert_not_called()

    def test_singularity_pause_keeps_session_but_invalidates_old_target(self):
        pending=Future()
        state=NS(guard_state='running',active=True,revision=12,approved_target=np.ones(7),
                 pending=pending,publish_twist=Mock(),publish_ghost=Mock(),stop_tracking=Mock())
        Backend.on_guard(state,NS(data='{"state":"paused","reason":"奇异位形暂停"}'))
        self.assertTrue(state.active)
        self.assertEqual(state.pause_revision,12)
        self.assertIsNone(state.approved_target)
        self.assertTrue(pending.cancelled())
        state.stop_tracking.assert_not_called()
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0],np.zeros(6))

    def recovery_state(self, condition=10.):
        actual=np.array([0.,0.,0.,0.,0.,0.,1.]);target=actual.copy();target[0]=.004
        config=load_config(Path(__file__).resolve().parents[1]/'scripts/task/servo_tracking/config/config.yaml')
        gate=Mock();gate.call_async.return_value=Future()
        return NS(resume_future=None,approved_target=target,valid=True,valid_revision=13,revision=13,
                  pause_revision=12,last_resume_attempt=0.,model=NS(condition_number=Mock(return_value=condition),
                  jacobian=Mock(return_value=np.eye(6)),check_joints=Mock()),config=config,scale=.5,
                  gate=gate,resume_done=Mock(),guard_state='paused'),actual

    def test_pause_requires_new_verified_target_before_resynchronization(self):
        state,actual=self.recovery_state()
        state.pause_revision=state.revision
        Backend.maybe_resume(state,np.zeros(6),actual)
        state.gate.call_async.assert_not_called()
        state.pause_revision-=1
        Backend.maybe_resume(state,np.zeros(6),actual)
        self.assertTrue(state.gate.call_async.call_args.args[0].data)
        self.assertIsNotNone(state.resume_future)

    def test_actual_singular_pose_does_not_add_a_condition_number_resume_block(self):
        state,actual=self.recovery_state(float('inf'))
        Backend.maybe_resume(state,np.zeros(6),actual)
        state.gate.call_async.assert_called_once()
        state.model.condition_number.assert_not_called()

    def test_near_singularity_direction_is_left_to_native_servo(self):
        state,actual=self.recovery_state()
        state.model.condition_number.return_value=25.
        Backend.maybe_resume(state,np.zeros(6),actual)
        state.gate.call_async.assert_called_once()
        state.model.jacobian.assert_not_called()
        state.model.check_joints.assert_not_called()

    def test_paused_session_never_publishes_motion_even_with_an_approved_target(self):
        recovery,actual=self.recovery_state()
        state=NS(heartbeat=Mock(),switched=True,actual=Mock(return_value=(np.zeros(6),actual)),
                 target=recovery.approved_target,approved_target=recovery.approved_target,active=True,
                 dragging=False,panel_seen=time.monotonic(),guard_state='paused',maybe_resume=Mock(),
                 publish_twist=Mock(),stop_tracking=Mock())
        Backend.tick(state)
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0],np.zeros(6))
        state.maybe_resume.assert_called_once()

    def start_state(self):
        config=load_config(Path(__file__).resolve().parents[1]/'scripts/task/servo_tracking/config/config.yaml')
        actual=np.array([.3,0.,.4,0.,0.,0.,1.]);target=actual.copy();target[0]+=.04
        gate=Mock();gate.call_async.return_value=Future()
        state=NS(active=False,panel_seen=time.monotonic(),switched=True,ik=Mock(),gate=gate,
                 actual=Mock(return_value=(np.zeros(6),actual)),model=Mock(),config=config,
                 target=target.copy(),approved_target=target.copy(),valid=True,valid_revision=8,
                 revision=8,aligned_revision=-1,align=Mock(),start_done=Mock(),reachability='可达')
        state.model.condition_number.return_value=10.
        state.begin_tracking=MethodType(Backend.begin_tracking,state)
        return state,target

    def test_start_keeps_existing_reachable_target_instead_of_aligning(self):
        state,target=self.start_state()
        result=Backend.start(state,None,NS())
        self.assertTrue(result.success)
        state.align.assert_not_called()
        np.testing.assert_array_equal(state.target,target)
        self.assertTrue(state.gate.call_async.call_args.args[0].data)

    def test_start_rejects_invalid_target_without_moving_it_or_opening_gate(self):
        state,target=self.start_state()
        state.valid=False;state.reachability='目标不可达或碰撞'
        result=Backend.start(state,None,NS())
        self.assertFalse(result.success)
        self.assertIn('重置',result.message)
        state.align.assert_not_called()
        state.gate.call_async.assert_not_called()
        np.testing.assert_array_equal(state.target,target)

    def test_start_waits_for_current_revision_ik_before_opening_gate(self):
        state,target=self.start_state()
        state.valid=False;state.valid_revision=7
        result=Backend.start(state,None,NS())
        self.assertTrue(result.success)
        self.assertTrue(state.start_pending)
        state.gate.call_async.assert_not_called()
        future=Future();state.pending=future;state.publish_ghost=Mock()
        state.model.pose.return_value=target
        future.set_result(NS(error_code=NS(val=1,SUCCESS=1),
                             solution=NS(joint_state=NS(name=JOINTS,position=[0.]*6))))
        Backend.ik_done(state,future,state.revision)
        self.assertFalse(state.start_pending)
        self.assertTrue(state.gate.call_async.call_args.args[0].data)
        np.testing.assert_array_equal(state.target,target)

    def test_reset_current_zero_pose_is_a_hold_not_a_margin_error(self):
        measured=(np.zeros(6),np.array([.2,0.,.3,0.,0.,0.,1.]))
        state=NS(actual=Mock(return_value=measured),raw_actual=Mock(return_value=measured),
                 revision=4,publish_ghost=Mock(),marker=Mock())
        Backend.align(state)
        self.assertTrue(state.valid)
        self.assertEqual(state.valid_revision,state.revision)
        self.assertEqual(state.reachability,'当前位置')
        np.testing.assert_array_equal(state.publish_ghost.call_args.args[0],np.zeros(6))

    def test_target_robot_state_is_yellow_visualization_with_complete_joint_solution(self):
        state=NS(get_clock=Mock(),revision=12,ghost_pub=Mock(),model=NS(link_names=['base_link','link1','link6']))
        state.get_clock.return_value.now.return_value.to_msg.return_value=Time(sec=1)
        q=[0.,1.4,-1.2,.3,1.,0.]
        Backend.publish_ghost(state,q)
        msg=state.ghost_pub.publish.call_args.args[0]
        self.assertFalse(msg.hide)
        self.assertEqual(msg.state.joint_state.name,JOINTS)
        self.assertEqual(list(msg.state.joint_state.position),q)
        self.assertTrue(all(h.color.r>.9 and h.color.g>.8 and h.color.b<.1 for h in msg.highlight_links))
        self.assertEqual(state.ghost_revision,12)
        Backend.publish_ghost(state)
        self.assertTrue(state.ghost_pub.publish.call_args.args[0].hide)
        self.assertEqual(state.ghost_revision,-1)


if __name__=='__main__':unittest.main()
