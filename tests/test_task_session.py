"""Session workflow checks with message fixtures; no ROS nodes or hardware."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock
import yaml
try:
    from scripts.task.multi_point.session import TaskSession
    from tests import test_task_ros_messages as helpers
    ROS=True
except ImportError:
    ROS=False

@unittest.skipUnless(ROS,'Use the ROS runtime wrapper')
class SessionTests(unittest.TestCase):
    def setUp(self):
        from scripts.task.common.schema import DEFAULTS
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.client=helpers.TaskMessageTests().client()
        self.client.settings=dict(DEFAULTS)
        self.client.activity_generation=0
        self.client.cancel_requested=threading.Event()
        self.client.begin_round=Mock()
        self.client.connect=Mock()
        self.client.replay_preview=Mock()
        self.client.clear_preview=Mock()
        self.client.restore_task_control=Mock()
        self.session=TaskSession(self.client,self.root/'runs')
        self.path=self.root/'task.yaml'
        self.write(.1,.2)

    def tearDown(self):
        self.session.close();self.temp.cleanup()

    def write(self,*positions):
        self.path.write_text(yaml.safe_dump({'points':[dict(name=f'p{i}',type='joint',joints_rad=[q]*6) for i,q in enumerate(positions)]}))

    def command(self,command,path=''):
        accepted,message=self.session.submit(command,str(path))
        self.assertTrue(accepted,message)
        self.session.jobs.join()
        return self.session.status()

    def test_two_rounds_replan_from_previous_real_endpoint(self):
        self.command('load',self.path)
        self.assertEqual(self.client.executions,0)
        self.command('plan')
        self.assertEqual(self.client.starts[0],(0.,)*6)
        self.command('execute')
        self.assertEqual(self.client.q,(.2,)*6)
        self.assertEqual(self.session.status()['state'],'IDLE')
        self.assertFalse(self.session.submit('execute')[0])
        self.command('plan')
        self.assertEqual(self.client.starts[2],(.2,)*6)
        self.command('execute')
        self.assertEqual(self.client.executions,4)
        self.assertEqual(len(list((self.root/'runs'/'rounds').iterdir())),2)

    def test_out_of_bounds_feedback_is_diagnosed_without_clamping_or_motion(self):
        from scripts.task.common.moveit_client import MoveItClient
        from scripts.task.common.schema import JOINTS
        self.client.limits={name:(-3.2,3.2) for name in JOINTS}
        self.client.limits['joint2']=(0.,3.1415926)
        self.client.q=(0.,-.045361,0.,0.,0.,0.)
        self.client.check_state=lambda q:MoveItClient.check_joints(self.client,q)
        self.command('load',self.path)
        status=self.command('plan')
        self.assertEqual(status['state'],'START_INVALID')
        self.assertEqual(status['summary'],'起点越界 · joint2 -2.60°')
        self.assertFalse(status['can_execute'])
        self.assertFalse(self.session.submit('execute')[0])
        self.assertEqual(self.client.q[1],-.045361)
        self.assertEqual(self.client.starts,[])
        self.assertEqual((self.client.executions,self.client.stops),(0,0))
        output=Path(status['last_output'])
        result=json.loads((output/'result.json').read_text())
        feedback=json.loads((output/'initial_feedback.json').read_text())
        self.assertEqual(result['error_details']['source'],'real_feedback')
        self.assertEqual(result['initial_joints_rad'][1],-.045361)
        self.assertEqual(feedback['limits_rad']['joint2'],[0.,3.1415926])
        # Once the supplied measurement is valid, a new preview can be made.
        # This is a fixture update, never a robot recovery command.
        self.client.q=(0.,.1,0.,0.,0.,0.)
        status=self.command('plan')
        self.assertEqual(status['state'],'READY')
        self.assertEqual(status['summary'],'')
        self.assertEqual(self.client.starts[0],self.client.q)
        self.assertEqual(self.client.executions,0)

    def test_feedback_failure_has_its_own_status_and_structured_log(self):
        from scripts.task.common.robot_feedback import FeedbackUnavailableError
        self.command('load',self.path)
        error=FeedbackUnavailableError(dict(kind='feedback_unavailable',reason='source_stale',
            receive_age_seconds=.1,source_age_seconds=.6,timeout_seconds=.5))
        self.client.stable=Mock(side_effect=error)
        status=self.command('plan')
        self.assertEqual(status['state'],'FEEDBACK_UNAVAILABLE')
        self.assertFalse(status['can_execute'])
        self.assertEqual(self.client.executions,0)
        result=json.loads((Path(status['last_output'])/'result.json').read_text())
        self.assertEqual(result['error_details']['reason'],'source_stale')

    def test_config_switch_and_file_edits_are_reloaded(self):
        self.command('load_plan',self.path)
        other=self.root/'other.yaml'
        other.write_text(yaml.safe_dump({'points':[dict(name='other',type='joint',joints_rad=[.3]*6)]}))
        self.client.clear_preview.reset_mock()
        self.command('load',other)
        self.assertFalse(self.session.status()['can_execute'])
        self.assertFalse(self.session.status()['preview_visible'])
        self.client.clear_preview.assert_called_once()
        self.command('plan');self.command('execute')
        self.assertEqual(self.client.q,(.3,)*6)
        other.write_text(yaml.safe_dump({'points':[dict(name='edited',type='joint',joints_rad=[.4]*6)]}))
        self.command('plan');self.command('execute')
        self.assertEqual(self.client.q,(.4,)*6)

    def test_manual_motion_invalidates_preview_but_does_not_stop_arm(self):
        self.command('load_plan',self.path)
        self.client.q=(.05,)*6
        self.client.clear_preview.reset_mock()
        self.session.poll()
        self.assertFalse(self.session.status()['can_execute'])
        self.assertFalse(self.session.status()['preview_visible'])
        self.client.clear_preview.assert_called_once()
        self.assertEqual(self.client.stops,0)
        self.command('plan')
        self.assertEqual(self.client.starts[-2],(.05,)*6)
        self.client.activity_generation+=1
        self.session.poll()
        self.assertFalse(self.session.status()['can_execute'])

    def test_manual_releases_visualization_without_control_commands(self):
        self.command('load_plan',self.path)
        self.assertTrue(self.session.status()['preview_visible'])
        self.client.clear_preview.reset_mock()
        status=self.command('manual')
        self.assertEqual(status['state'],'MANUAL')
        self.assertFalse(status['preview_visible'])
        self.assertFalse(status['can_execute'])
        self.client.clear_preview.assert_called_once()
        self.assertEqual(self.client.stops,0)
        self.assertEqual(self.client.executions,0)
        self.command('plan')
        self.assertTrue(self.session.status()['preview_visible'])

    def test_stop_lock_requires_explicit_recovery_not_mode_switch(self):
        self.command('load_plan',self.path)
        self.client.task_gate_closed=True
        self.command('manual')
        self.client.restore_task_control.assert_not_called()
        self.command('plan')
        self.assertFalse(self.session.submit('execute')[0])
        self.client.restore_task_control.side_effect=lambda:setattr(self.client,'task_gate_closed',False)
        self.command('recover')
        self.client.restore_task_control.assert_called_once()
        self.assertFalse(self.session.status()['can_execute'])
        self.assertFalse(self.session.status()['recovery_required'])
        self.command('plan');self.command('execute')
        self.assertEqual(self.client.executions,2)

    def test_ten_points_stop_at_two_recover_and_restart_all_points(self):
        self.write(*[i/10 for i in range(1,11)])
        self.command('load_plan',self.path)
        execute=self.client.execute
        def stop_on_two(*args):
            execute(*args)
            if self.client.executions==2:
                self.client.q=(.2,)*6
                raise InterruptedError('Task canceled')
        stop=self.client.stop
        def close_gate():
            self.client.task_gate_closed=True
            return stop()
        self.client.execute=stop_on_two
        self.client.stop=close_gate
        status=self.command('execute')
        self.assertEqual(status['state'],'ERROR')
        self.assertTrue(status['recovery_required'])
        self.assertFalse(status['can_execute'])
        self.client.restore_task_control.side_effect=lambda:setattr(self.client,'task_gate_closed',False)
        self.command('recover')
        self.assertEqual(self.client.executions,2)
        self.assertFalse(self.session.submit('execute')[0])
        self.client.execute=execute
        self.command('plan')
        self.assertEqual(self.client.starts[-10],(.2,)*6)
        status=self.command('execute')
        self.assertEqual(status['state'],'IDLE')
        self.assertEqual(self.client.executions,12)
        self.assertEqual([t.joint_trajectory.points[-1].positions[0] for t in self.client.sent[-10:]],
                         [i/10 for i in range(1,11)])

    def test_invalid_load_does_not_keep_previous_execution_confirmation(self):
        self.command('load_plan',self.path)
        bad=self.root/'bad.yaml';bad.write_text('points: [')
        status=self.command('load',bad)
        self.assertFalse(status['has_config'])
        self.assertFalse(status['can_execute'])
        self.assertFalse(self.session.submit('execute')[0])
        self.command('load',self.path)
        self.assertTrue(self.session.status()['has_config'])

    def test_failure_returns_to_session_and_never_retries(self):
        self.command('load_plan',self.path)
        self.client.execute=Mock(side_effect=RuntimeError('test failure'))
        status=self.command('execute')
        self.assertEqual(status['state'],'ERROR')
        self.assertFalse(status['can_execute'])
        self.client.execute.assert_called_once()
        self.assertEqual(self.client.stops,1)
        self.command('plan')
        self.assertTrue(self.session.status()['can_execute'])

    def test_planning_failure_keeps_config_but_clears_old_preview(self):
        self.command('load_plan',self.path)
        self.client.plan=Mock(side_effect=RuntimeError('goal sampling failed'))
        status=self.command('plan')
        self.assertEqual(status['state'],'ERROR')
        self.assertTrue(status['has_config'])
        self.assertIn('第 1/2 点 p0 规划失败',status['message'])
        self.assertFalse(status['can_execute'])
        self.assertFalse(status['preview_visible'])
        self.assertEqual(self.client.executions,0)
        self.assertEqual(self.client.stops,0)
        self.client.plan.assert_called_once()

    def test_busy_commands_serialized_and_stop_interrupts_work(self):
        entered=threading.Event()
        original=self.client.connect
        def wait(**kwargs):
            entered.set()
            self.client.cancel_requested.wait(2)
            raise InterruptedError('cancelled')
        self.client.connect=wait
        self.assertTrue(self.session.submit('load',str(self.path))[0])
        self.assertTrue(entered.wait(1))
        self.assertFalse(self.session.submit('load',str(self.path))[0])
        self.assertFalse(self.session.submit('execute')[0])
        self.assertTrue(self.session.submit('stop')[0])
        self.session.jobs.join()
        self.assertFalse(self.session.status()['busy'])
        self.assertEqual(self.client.executions,0)
        self.client.connect=original
        self.command('load',self.path)
        self.assertFalse(self.session.submit('stop')[0])

    def test_preview_cleanup_failure_does_not_kill_worker(self):
        self.client.clear_preview.side_effect=RuntimeError('publisher unavailable')
        status=self.command('load',self.path)
        self.assertEqual(status['state'],'ERROR')
        self.assertTrue(self.session.worker.is_alive())
        self.client.clear_preview.side_effect=None
        status=self.command('load',self.path)
        self.assertEqual(status['state'],'LOADED')

@unittest.skipUnless(ROS,'Use the ROS runtime wrapper')
class SessionRuntimeTests(unittest.TestCase):
    def test_rviz_config_separates_measured_solid_and_original_color_ghosts(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from scripts.task.common.runtime import Runtime
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'config').mkdir()
            (root/'config'/'moveit.rviz').write_text(yaml.safe_dump({'Visualization Manager':{'Displays':[
                {'Class':'moveit_rviz_plugin/MotionPlanning','Planning Request':{'Interactive Marker Size':.1}}]}}))
            client=Mock(urdf='urdf',srdf='srdf')
            client.call.side_effect=[SimpleNamespace(result=SimpleNamespace(names=[])),
                                    SimpleNamespace(values=[SimpleNamespace(type=9,string_array_value=['ompl'])])]
            runtime=Runtime(client,root,root)
            runtime.session_panel=True
            process=Mock();process.poll.return_value=None
            def spawn(*args):
                runtime.processes.append(process);return process
            with patch('scripts.task.common.runtime.get_package_share_directory',return_value=temp), \
                 patch.object(runtime,'read_rviz_parameters',return_value={'planning_pipelines':['ompl'],
                    'default_planning_pipeline':'ompl','robot_description_kinematics':{'arm':{}}}), \
                 patch.object(runtime,'spawn',side_effect=spawn) as launch, \
                 patch('scripts.task.common.runtime.time.sleep'):
                runtime.start_rviz()
            displays=yaml.safe_load((root/'task.rviz').read_text())['Visualization Manager']['Displays'][0]['Displays']
            official,ghost=displays[:2]
            self.assertTrue(official['Scene Robot']['Show Robot Visual'])
            self.assertEqual(official['Scene Robot']['Robot Alpha'],1.0)
            self.assertEqual(official['Class'],'moveit_rviz_plugin/MotionPlanning')
            self.assertEqual(official['Planning Request']['Goal State Alpha'],.45)
            self.assertEqual(official['Planning Request']['Start State Alpha'],.45)
            self.assertTrue(official['Planning Request']['Query Goal State'])
            self.assertEqual(official['Planning Request']['Interactive Marker Size'],.1)
            self.assertEqual(official['Planned Path']['Robot Alpha'],.35)
            self.assertFalse(official['Planned Path']['Color Enabled'])
            self.assertFalse(official['Planned Path']['Loop Animation'])
            panels=yaml.safe_load((root/'task.rviz').read_text())['Panels']
            self.assertTrue(next(p for p in panels if p['Class']=='piper_multi_point_rviz/TaskPanel')['Default Left Layout'])
            self.assertEqual(ghost['Robot Alpha'],.35)
            self.assertFalse(ghost['Color Enabled'])
            self.assertIn('joint_states:=/feedback/joint_states',launch.call_args.args[1])

    def test_console_ctrl_c_leaves_busy_session_through_cancellation_cleanup(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from scripts.task.multi_point.console import run_session
        client=Mock(cancel_requested=threading.Event())
        shutdown=threading.Event()
        session=Mock()
        session.status.return_value={'busy':True}
        session.poll.side_effect=KeyboardInterrupt
        api=Mock()
        with patch('scripts.task.multi_point.console.TaskSession',return_value=session), \
             patch('scripts.task.multi_point.console.SessionAPI',return_value=api):
            run_session(client,Mock(),SimpleNamespace(directory=Path('/tmp')),
                        SimpleNamespace(scene=None,config=None),shutdown)
        self.assertTrue(shutdown.is_set())
        self.assertTrue(client.cancel_requested.is_set())
        session.close.assert_called_once()
        api.close.assert_called_once()
        session.poll.assert_called_once()

    def test_ctrl_c_exits_busy_session_and_repeated_signals_do_not_break_cleanup(self):
        import signal
        from unittest.mock import patch
        from scripts.task.multi_point.run import main
        client=Mock(cancel_requested=threading.Event())
        runtime=Mock()
        before={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
        calls=[]
        def session(client,runtime,log,args,shutdown):
            signal.raise_signal(signal.SIGINT)
            self.assertTrue(shutdown.is_set())
            self.assertTrue(client.cancel_requested.is_set())
            calls.append('session_cancelled')
        def close():
            signal.raise_signal(signal.SIGINT)
            signal.raise_signal(signal.SIGTERM)
            calls.append('processes_closed')
        runtime.close.side_effect=close
        client.close.side_effect=lambda:calls.append('client_closed')
        with tempfile.TemporaryDirectory() as temp, \
             patch('scripts.task.common.moveit_client.MoveItClient',return_value=client), \
             patch('scripts.task.common.runtime.Runtime',return_value=runtime), \
             patch('scripts.task.multi_point.console.run_session',side_effect=session):
            self.assertEqual(main(['--rviz','--interactive','--output-dir',temp]),0)
        self.assertEqual(calls,['session_cancelled','processes_closed','client_closed'])
        self.assertEqual(before,{sig:signal.getsignal(sig) for sig in before})

    def test_failed_process_cleanup_still_closes_ros_and_restores_signals(self):
        import signal
        from unittest.mock import patch
        from scripts.task.multi_point.run import main
        client=Mock();runtime=Mock()
        runtime.close.side_effect=RuntimeError('cleanup failed')
        before={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
        with tempfile.TemporaryDirectory() as temp, \
             patch('scripts.task.common.moveit_client.MoveItClient',return_value=client), \
             patch('scripts.task.common.runtime.Runtime',return_value=runtime), \
             patch('scripts.task.multi_point.console.run_session'):
            with self.assertRaisesRegex(RuntimeError,'cleanup failed'):
                main(['--rviz','--interactive','--output-dir',temp])
        client.close.assert_called_once()
        self.assertEqual(before,{sig:signal.getsignal(sig) for sig in before})

    def test_runtime_cleanup_continues_after_one_group_fails(self):
        from unittest.mock import patch
        from scripts.task.common.runtime import Runtime
        with tempfile.TemporaryDirectory() as temp:
            runtime=Runtime(Mock(),'.',temp)
            runtime.processes=[Mock(pid=100),Mock(pid=101)]
            stream=Mock();runtime.streams=[stream]
            with patch('scripts.task.common.runtime.stop_process_group',side_effect=[
                    OSError('test failure'),dict(pid=100,remaining_pids=[],errors=[])]) as stop:
                with self.assertRaisesRegex(RuntimeError,'进程清理未完成'): runtime.close()
            self.assertEqual(stop.call_count,2)
            stream.close.assert_called_once()
            self.assertEqual(len(json.loads((Path(temp)/'process_exit.json').read_text())),2)
            runtime.close()  # Idempotent: never signal a reused group ID.

    def test_existing_driver_starts_only_official_planning_with_actual_tcp(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from scripts.task.common.runtime import Runtime
        from rcl_interfaces.msg import ParameterValue
        client=Mock()
        client.call.return_value=SimpleNamespace(values=[
            ParameterValue(type=4,string_value='piper_l'),ParameterValue(type=4,string_value='none'),
            ParameterValue(type=4,string_value='left'),ParameterValue(type=8,double_array_value=[.0,.0,.1,.0,.0,.0])])
        runtime=Runtime(client,'.','.')
        with patch.object(runtime,'spawn') as spawn:
            runtime.start_planning_for_existing_driver()
        command=spawn.call_args.args[1]
        self.assertIn('agx_arm_moveit',command)
        self.assertIn('demo.launch.py',command)
        self.assertIn('tcp_offset:=[0.0, 0.0, 0.1, 0.0, 0.0, 0.0]',command)
        self.assertNotIn('agx_arm_ctrl',command)
        self.assertIn('follow:=true',command)

    def test_uninitialized_parameter_does_not_drop_planner_configuration(self):
        from types import SimpleNamespace
        from scripts.task.common.runtime import Runtime
        from rcl_interfaces.msg import ParameterValue
        client=Mock()
        values={'default_planning_pipeline':ParameterValue(type=4,string_value='ompl'),
                'empty_array':ParameterValue(type=9,string_array_value=[]),
                'planning_pipelines':ParameterValue(type=9,string_array_value=['ompl','pilz_industrial_motion_planner'])}
        def read(service,request):
            return SimpleNamespace(values=[] if 'unset' in request.names else [values[n] for n in request.names])
        client.call.side_effect=read
        result=Runtime(client,'.','.').read_rviz_parameters(['default_planning_pipeline','unset','planning_pipelines','empty_array'])
        self.assertEqual(result,{'default_planning_pipeline':'ompl','planning_pipelines':['ompl','pilz_industrial_motion_planner']})

    def test_clear_preview_cannot_be_replayed_and_never_touches_controller(self):
        from types import SimpleNamespace
        from scripts.task.common.moveit_client import MoveItClient
        client=SimpleNamespace(preview_message=object(),target_message=object(),display=Mock(),targets=Mock())
        MoveItClient.clear_preview(client)
        self.assertEqual(len(client.display.publish.call_args.args[0].trajectory),0)
        self.assertEqual(client.targets.publish.call_args.args[0].markers[0].action,3)
        MoveItClient.replay_preview(client)
        self.assertEqual(client.display.publish.call_count,1)

    def test_begin_round_clears_terminal_state_but_rejects_active_or_pending_motion(self):
        from types import SimpleNamespace
        from scripts.task.common.moveit_client import MoveItClient
        client=SimpleNamespace(tick=lambda:None,external_status=[],pending_goal=None,active_goal=None,
                               execution_result_future=None,motion_submitted=True)
        MoveItClient.begin_round(client)
        self.assertFalse(client.motion_submitted)
        client.external_status=[SimpleNamespace(status=2)]
        with self.assertRaises(RuntimeError):MoveItClient.begin_round(client)
        client.external_status=[];client.pending_goal=Mock();client.pending_goal.done.return_value=False
        with self.assertRaises(RuntimeError):MoveItClient.begin_round(client)
        client.pending_goal=None;client.active_goal=Mock(accepted=True)
        client.execution_result_future=Mock();client.execution_result_future.done.return_value=False
        with self.assertRaises(RuntimeError):MoveItClient.begin_round(client)
        client.execution_result_future.done.return_value=True
        MoveItClient.begin_round(client)
        self.assertIsNone(client.active_goal)
        handle=Mock(accepted=True)
        handle.get_result_async.return_value.done.return_value=False
        client.pending_goal=Mock()
        client.pending_goal.done.return_value=True
        client.pending_goal.result.return_value=handle
        with self.assertRaises(RuntimeError):MoveItClient.begin_round(client)

if __name__=='__main__':unittest.main()
