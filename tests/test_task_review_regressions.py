"""Cancellation and failure-path regressions; no nodes, CAN or physical motion."""
import json
import tempfile
import threading
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    from scripts.task.common.moveit_client import MoveItClient
    from scripts.task.common.ros_context import RosContext, fk_value
    from scripts.task.common.schema import DEFAULTS, validate_config
    from scripts.task.common.robot_feedback import Sample
    from moveit_msgs.msg import RobotTrajectory
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper')
class ReviewRegressions(unittest.TestCase):
    def test_completed_future_still_observes_stop(self):
        future = Future(); future.set_result(object())
        with self.assertRaises(InterruptedError):
            RosContext.wait(SimpleNamespace(), future, tick=Mock(side_effect=InterruptedError('stop')))

    def test_stop_during_service_discovery_prevents_request(self):
        tick = Mock(side_effect=[None, InterruptedError('stop')])
        client = Mock(); client.wait_for_service.return_value = True
        with self.assertRaises(InterruptedError):
            RosContext.call(SimpleNamespace(wait=Mock()), client, object(), tick=tick)
        client.call_async.assert_not_called()

    def test_execution_rechecks_state_after_action_discovery(self):
        for change in ('cancel', 'active', 'moved'):
            with self.subTest(change=change):
                c = SimpleNamespace(settings=dict(DEFAULTS), cancel_requested=threading.Event(),
                    feedback=Mock(), external_status=[], execution=Mock(), motion_submitted=False,
                    stable=Mock(return_value=SimpleNamespace(q=(0.,)*6)),
                    current=Mock(return_value=SimpleNamespace(q=(0.,)*6)))
                c.tick = lambda health=False: MoveItClient.tick(c, health)
                def ready(**kwargs):
                    if change == 'cancel': c.cancel_requested.set()
                    if change == 'active': c.external_status = [SimpleNamespace(status=2)]
                    if change == 'moved': c.current.return_value.q = (.1,)*6
                    return True
                c.execution.wait_for_server.side_effect = ready
                with self.assertRaises((InterruptedError, RuntimeError)):
                    MoveItClient.execute(c, RobotTrajectory(), [0.]*6, 1.)
                c.execution.send_goal_async.assert_not_called()
                self.assertFalse(c.motion_submitted)

    def test_fk_nonfinite_position_is_rejected(self):
        from moveit_msgs.srv import GetPositionFK
        from geometry_msgs.msg import PoseStamped
        reply = GetPositionFK.Response(); reply.error_code.val = 1
        reply.fk_link_names = ['tcp_link']
        pose = PoseStamped(); pose.header.frame_id = 'base_link'; pose.pose.orientation.w = 1.
        reply.pose_stamped = [pose]
        for value in (float('nan'), float('inf'), -float('inf')):
            pose.pose.position.x = value
            with self.assertRaises(ValueError): fk_value(reply, 'base_link', 'tcp_link')

    def test_fk_wait_receives_task_cancellation_callback(self):
        c = SimpleNamespace(fk_client=object(), tick=Mock(), call=Mock())
        with patch('scripts.task.common.ros_context.fk_value'):
            RosContext.fk(c, [0.]*6, 'base_link', 'tcp_link')
        self.assertIs(c.call.call_args.kwargs['tick'], c.tick)

    def test_logging_failure_never_prevents_stop_or_masks_original_failure(self):
        from scripts.task.common.run_logger import RunLog
        from scripts.task.multi_point.task import MultiPointTask
        from tests.test_task_ros_messages import TaskMessageTests
        for result_log_fails in (False, True):
            with self.subTest(result_log_fails=result_log_fails), tempfile.TemporaryDirectory() as temp:
                helper = TaskMessageTests(); client = helper.client()
                client.execute = Mock(side_effect=RuntimeError('execution failure'))
                log = RunLog(temp); event, write = log.event, log.write
                def failing_event(state, **fields):
                    if state == 'FAILED':
                        self.assertEqual(client.stops, 1)
                        raise OSError('disk full')
                    return event(state, **fields)
                def failing_write(name, value):
                    if result_log_fails and name == 'result.json': raise OSError('disk full')
                    return write(name, value)
                log.event, log.write = failing_event, failing_write
                with self.assertRaisesRegex(RuntimeError, 'execution failure'):
                    MultiPointTask(client, helper.config(), {}, log, True).run()
                self.assertEqual(client.stops, 1)
                segment = json.loads((log.directory/'001_a.json').read_text())
                self.assertEqual(segment['execution'], 'submitted')

    def test_dwell_checks_requested_joint_and_tcp_not_only_endpoint(self):
        point = dict(name='pose', type='pose', frame_id='base_link',
                     position_m=[0.,0.,0.], orientation_xyzw=[0.,0.,0.,1.])
        for mode in ('position', 'angle', 'joint', 'stale'):
            with self.subTest(mode=mode):
                c = SimpleNamespace(settings=dict(DEFAULTS), tick=Mock(),
                    current=Mock(return_value=Sample((0.,)*6, 1., 0.)),
                    fk=Mock(return_value=dict(position_m=[.01,0.,0.], orientation_xyzw=[0.,0.,0.,1.])))
                target = point
                if mode == 'angle': c.fk.return_value = dict(position_m=[0.,0.,0.], orientation_xyzw=[0.,0.,1.,0.])
                if mode == 'joint': target = dict(name='joint',type='joint',joints_rad=[.021]*6)
                if mode == 'stale': c.current.return_value = Sample((0.,)*6, 1., -1.)
                with patch('scripts.task.common.moveit_client.time.monotonic', return_value=0.), \
                     self.assertRaisesRegex(RuntimeError, 'during dwell'):
                    MoveItClient.hold(c, [0.]*6, .5, target)

    def test_zero_dwell_still_observes_stop(self):
        c = SimpleNamespace(tick=Mock(side_effect=InterruptedError('stop')))
        with self.assertRaises(InterruptedError): MoveItClient.hold(c, [0.]*6, 0.)

    def test_return_label_is_reserved_only_when_return_is_enabled(self):
        config = dict(points=[dict(name='return_to_initial', type='joint', joints_rad=[0.]*6)])
        validate_config(config)
        config['settings'] = dict(return_to_initial=True)
        with self.assertRaises(ValueError): validate_config(config)

    def test_api_startup_failure_closes_session_worker(self):
        from scripts.task.multi_point.console import run_session
        session = Mock()
        with patch('scripts.task.multi_point.console.TaskSession', return_value=session), \
             patch('scripts.task.multi_point.console.SessionAPI', side_effect=RuntimeError('service init failed')), \
             self.assertRaisesRegex(RuntimeError, 'service init failed'):
            run_session(Mock(), Mock(), SimpleNamespace(directory=Path('/tmp')),
                        SimpleNamespace(scene=None), threading.Event())
        session.close.assert_called_once()

    def test_existing_auto_gate_rejected_before_any_stack_launch(self):
        from scripts.task.common.runtime import Runtime
        runtime = Runtime(Mock(), '.', '.')
        with patch.object(runtime, 'acquire'), patch.object(runtime, 'names', return_value={'control_gate'}), \
             patch.object(runtime, 'spawn') as spawn, patch('scripts.task.common.runtime.time.sleep'), \
             self.assertRaisesRegex(RuntimeError, 'auto_control_gate:=false'):
            runtime.prepare('auto', False)
        spawn.assert_not_called()
        runtime.client.connect.assert_not_called()


if __name__ == '__main__': unittest.main()
