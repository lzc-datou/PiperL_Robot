"""Message/workflow tests without ROS nodes, CAN access or simulated motion."""
import copy
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
try:
    from moveit_msgs.msg import PlanningScene as Scene, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    ROS=True
except ImportError:
    ROS=False

@unittest.skipUnless(ROS,'Use the ROS runtime wrapper')
class TaskMessageTests(unittest.TestCase):
    def test_raw_feedback_uses_source_stamp_not_republish(self):
        from scripts.task.common.robot_feedback import Feedback
        from scripts.task.common.schema import JOINTS
        feedback=Feedback()
        msg=JointState()
        msg.name=JOINTS
        msg.position=[0.0]*6
        msg.header.stamp.sec=100
        with patch('time.monotonic',return_value=10): feedback.joint_callback(msg)
        with patch('time.monotonic',return_value=12): feedback.joint_callback(msg)
        with self.assertRaises(RuntimeError): feedback.get(0.5,now=12,wall=100.1)
        msg.header.stamp.sec=99
        feedback.joint_callback(msg)
        with self.assertRaisesRegex(RuntimeError,'backwards'): feedback.get(1,now=10,wall=100)
    def test_scene_fingerprint_ignores_robot_motion_but_detects_obstacles(self):
        from scripts.task.common.planning_scene import fingerprint,obstacle_message
        scene=Scene()
        before=fingerprint(scene)
        scene.robot_state.joint_state.position=[1.0]
        self.assertEqual(fingerprint(scene),before)
        scene.world.collision_objects=[obstacle_message(dict(id='a',shape='box',dimensions_m=[0.1]*3,position_m=[0.3,0.0,0.2],orientation_xyzw=[0.,0.,0.,1.],frame_id='base_link'))]
        self.assertNotEqual(fingerprint(scene),before)
        self.assertEqual(scene.world.collision_objects[0].id,'multi_point/a')
    def test_fk_request_contains_exact_snapshot_positions(self):
        from scripts.task.common.ros_context import fk_request,fk_value
        from moveit_msgs.srv import GetPositionFK
        from geometry_msgs.msg import PoseStamped
        q=[0.1,0.2,-0.3,0.,0.,0.]
        request=fk_request(q,'base_link','tcp_link')
        self.assertEqual(list(request.robot_state.joint_state.position),q)
        reply=GetPositionFK.Response()
        reply.error_code.val=1
        reply.fk_link_names=['tcp_link']
        p=PoseStamped()
        p.header.frame_id='base_link'
        p.pose.orientation.w=1.0
        reply.pose_stamped=[p]
        self.assertEqual(fk_value(reply,'base_link','tcp_link')['orientation_xyzw'],[0,0,0,1])
        with self.assertRaises(RuntimeError): fk_value(reply,'wrong','tcp_link')

    def test_local_fk_snapshot_checks_tcp_geometry_before_planning(self):
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        import xacro
        from scripts.task.common.run_logger import RunLog
        from scripts.task.common.schema import validate_config
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.servo_tracking.core import load_config
        from scripts.tools.monitoring.point_panel import LocalFK
        from scripts.tools.monitoring.point_snapshot import Capture, documents
        root = Path(__file__).resolve().parents[1]
        local = LocalFK(root, 'base_link', 'tcp_link')
        capture = Capture(q=(0.,)*6, received=(1.,)*6, captured_at='fixture', monotonic=1.)
        config = validate_config(documents(
            capture, local.pose(capture), kinematics_sha256=local.kinematics_sha256)['joint'])
        model_path = Path(get_package_share_directory('agx_arm_moveit'))/'config/agx_arm.urdf.xacro'
        offset = load_config(root/'scripts/task/servo_tracking/config/config.yaml')['tcp_offset']
        mappings = {'arm_type': 'piper_l', 'effector_type': 'none',
                    'tcp_offset_xyz': ' '.join(map(str, offset[:3])),
                    'tcp_offset_rpy': ' '.join(map(str, offset[3:]))}
        with tempfile.TemporaryDirectory() as temp:
            client = self.client()
            client.urdf = xacro.process_file(str(model_path), mappings=mappings).toxml()
            task = MultiPointTask(client, config, {'enabled': False, 'objects': []}, RunLog(temp))
            self.assertTrue(task.run()['success'])
            mappings['tcp_offset_xyz'] = ' '.join(map(str, (offset[0]+.05, *offset[1:3])))
            client.urdf = xacro.process_file(str(model_path), mappings=mappings).toxml()
            client.starts.clear()
            with self.assertRaisesRegex(RuntimeError, 'different robot model/TCP'):
                task.run()
            self.assertEqual(client.starts, [])

    def client(self,fail=False):
        from scripts.task.common.schema import JOINTS
        class Client:
            urdf='urdf';srdf='srdf';model_sha256='sha'
            def __init__(self):
                self.starts=[];self.executions=0;self.stops=0;self.preview=None
                self.q=(0.0,)*6
                self.sent=[]
                self.scene=SimpleNamespace(apply=lambda c:Scene(),get=lambda:Scene())
            def tick(self,*args): pass
            def model(self): return [self.urdf,self.srdf]
            def current(self): return SimpleNamespace(q=self.q)
            def stable(self,target=None,**kwargs):
                if target is not None: self.q=tuple(target)
                return self.current()
            def check_state(self,q): pass
            def fk(self,*args): return dict(position_m=[0.,0.,0.],orientation_xyzw=[0.,0.,0.,1.])
            def publish_targets(self,p): pass
            def publish_plan(self,start,trajectories): self.preview=(tuple(start),list(trajectories))
            def plan(self,point,start,*,publish=True):
                self.publish_flags=getattr(self,'publish_flags',[])+[publish]
                self.starts.append(tuple(start))
                if fail and len(self.starts)==2: raise RuntimeError('planning failure')
                trajectory=RobotTrajectory()
                trajectory.joint_trajectory.joint_names=JOINTS
                p=JointTrajectoryPoint(positions=point['joints_rad'])
                trajectory.joint_trajectory.points=[p]
                return trajectory,point['joints_rad'],1.0
            def execute(self,*args):
                self.executions+=1
                self.sent.append(args[0])
            def hold(self,*args): pass
            def stop(self): self.stops+=1;return {'physical_stop_confirmed':False}
        return Client()
    def config(self,returns=False):
        from scripts.task.common.schema import validate_config
        return validate_config({'settings':{'return_to_initial':returns},'points':[dict(name='a',type='joint',joints_rad=[0.1]*6),dict(name='b',type='joint',joints_rad=[0.2]*6)]})
    def test_plan_only_chains_predictions_without_execute_or_stop(self):
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        with tempfile.TemporaryDirectory() as temp:
            client=self.client()
            result=MultiPointTask(client,self.config(),{'enabled':False,'objects':[]},RunLog(temp)).run()
            self.assertTrue(result['success'])
            self.assertEqual(client.executions,0)
            self.assertEqual(client.stops,0)
            self.assertEqual(client.starts[1],(0.1,)*6)
            self.assertEqual(client.preview[0],(0.,)*6)
            self.assertEqual(len(client.preview[1]),2)
            self.assertEqual(client.publish_flags,[False,False])
            self.assertEqual(list(client.preview[1][0].joint_trajectory.points[-1].positions),[0.1]*6)
            self.assertEqual(list(client.preview[1][1].joint_trajectory.points[-1].positions),[0.2]*6)
    def test_execution_return_and_arrival(self):
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        with tempfile.TemporaryDirectory() as temp:
            client=self.client()
            result=MultiPointTask(client,self.config(True),{'enabled':False,'objects':[]},RunLog(temp),True).run()
            self.assertEqual(client.executions,3)
            self.assertIsNone(client.preview)
            self.assertEqual(client.publish_flags,[True,True,True])
            self.assertTrue(result['physical_arrival_checked'])
            self.assertEqual(client.q,(0.,)*6)
    def test_failure_stops_sequence_without_return(self):
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        with tempfile.TemporaryDirectory() as temp:
            client=self.client(True)
            with self.assertRaisesRegex(RuntimeError,'planning failure'):
                MultiPointTask(client,self.config(True),{'enabled':False,'objects':[]},RunLog(temp),True).run()
            self.assertEqual(client.executions,1)
            self.assertEqual(client.stops,1)
            self.assertEqual(len(client.starts),2)

    def test_failed_pose_preserves_resolved_rpy_orientation_and_error(self):
        import json
        from unittest.mock import Mock
        from moveit_msgs.msg import MoveItErrorCodes
        from scripts.task.common.moveit_client import PlanningError
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        from scripts.task.common.schema import validate_config
        config=validate_config({'points':[
            dict(name='a',type='joint',joints_rad=[0.1]*6),
            dict(name='b',type='pose',position_m=[.05,.2,.79],rpy=[90.,90.,0.])]})
        with tempfile.TemporaryDirectory() as temp:
            client=self.client()
            original_plan=client.plan
            def plan(point,start,**kwargs):
                if point['name']=='b':
                    raise PlanningError(MoveItErrorCodes(val=99999,source='OMPL',message='test goal failure'))
                return original_plan(point,start,**kwargs)
            client.plan=Mock(side_effect=plan)
            log=RunLog(temp)
            task=MultiPointTask(client,config,{},log)
            with self.assertRaisesRegex(RuntimeError,'第 2/2 点 b 规划失败.*FAILURE'):
                task.run()
            failure=json.loads((log.directory/'planning_failure.json').read_text())
            for actual, expected in zip(failure['point']['orientation_xyzw'], [.5,.5,-.5,.5]):
                self.assertAlmostEqual(actual, expected)
            self.assertNotIn('rpy',failure['point'])
            self.assertEqual(failure['start_joints_rad'],[.1]*6)
            self.assertEqual(failure['moveit_error'],dict(code=99999,name='FAILURE',source='OMPL',message='test goal failure'))
            self.assertTrue((log.directory/'002_scene.json').exists())
            self.assertEqual(json.loads((log.directory/'002_request.json').read_text())['point'],failure['point'])
            self.assertEqual(json.loads((log.directory/'resolved_points.json').read_text())[1],failure['point'])
            self.assertEqual(json.loads((log.directory/'initial_pose.json').read_text()),client.fk())
            self.assertEqual(json.loads((log.directory/'result.json').read_text())['planning_failure'],failure)
            self.assertEqual(config['points'][1]['orientation_xyzw'], failure['point']['orientation_xyzw'])
            self.assertIsNone(task.reviewed_plan)
            self.assertIsNone(client.preview)
            self.assertEqual(client.executions,0)
            self.assertEqual(client.stops,0)
            self.assertEqual(client.plan.call_count,2)

    def test_canceled_planning_remains_cancellation(self):
        from unittest.mock import Mock
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        with tempfile.TemporaryDirectory() as temp:
            client=self.client()
            client.plan=Mock(side_effect=InterruptedError('Task canceled'))
            log=RunLog(temp)
            with self.assertRaises(InterruptedError):
                MultiPointTask(client,self.config(),{},log).run()
            self.assertFalse((log.directory/'planning_failure.json').exists())
            self.assertEqual(client.executions,0)

@unittest.skipUnless(ROS,'Use the ROS runtime wrapper')
class RvizPreviewTests(unittest.TestCase):
    def test_owned_window_waits_until_closed_without_robot_calls(self):
        from unittest.mock import Mock
        from scripts.task.common.runtime import Runtime
        client=SimpleNamespace(cancel_requested=Mock())
        client.cancel_requested.wait.return_value=False
        runtime=Runtime(client,'.','.')
        runtime.rviz_process=Mock(returncode=0)
        runtime.rviz_process.poll.side_effect=[None,None,0]
        runtime.wait_for_rviz()
        self.assertEqual(client.cancel_requested.wait.call_count,2)

    def test_external_window_does_not_block(self):
        from scripts.task.common.runtime import Runtime
        Runtime(SimpleNamespace(),'.','.').wait_for_rviz()

    def test_viewing_interrupt_and_termination_end_normally(self):
        from unittest.mock import Mock
        from scripts.task.common.runtime import Runtime
        for interrupted in (False,True):
            with self.subTest(interrupted=interrupted):
                client=SimpleNamespace(cancel_requested=Mock())
                client.cancel_requested.wait.return_value=True
                if interrupted: client.cancel_requested.wait.side_effect=KeyboardInterrupt
                runtime=Runtime(client,'.','.')
                runtime.rviz_process=Mock()
                runtime.rviz_process.poll.return_value=None
                runtime.wait_for_rviz()
                client.cancel_requested.wait.assert_called_once_with(0.2)

    def test_window_failure_is_reported(self):
        from unittest.mock import Mock
        from scripts.task.common.runtime import Runtime
        runtime=Runtime(SimpleNamespace(),'.','.')
        runtime.rviz_process=Mock(returncode=1)
        runtime.rviz_process.poll.return_value=1
        with self.assertRaisesRegex(RuntimeError,'RViz exited with code 1'):
            runtime.wait_for_rviz()

    def test_main_keeps_resources_until_preview_ends_and_skips_wait_on_failure(self):
        from unittest.mock import Mock
        import yaml
        from pathlib import Path
        from scripts.task.multi_point.run import main
        for rviz,failed in ((True,False),(False,False),(True,True)):
            with self.subTest(rviz=rviz,failed=failed),tempfile.TemporaryDirectory() as temp:
                path=Path(temp)/'config.yaml'
                path.write_text(yaml.safe_dump({'points':[dict(name='p1',type='joint',joints_rad=[0.]*6)]}))
                calls=[]
                client=Mock()
                runtime=Mock()
                task=Mock()
                client.close.side_effect=lambda:calls.append('client_close')
                runtime.close.side_effect=lambda:calls.append('runtime_close')
                runtime.wait_for_rviz.side_effect=lambda:calls.append('preview')
                def run():
                    calls.append('task')
                    if failed: raise RuntimeError('planning failed')
                task.run.side_effect=run
                with patch('scripts.task.common.moveit_client.MoveItClient',return_value=client), \
                     patch('scripts.task.common.runtime.Runtime',return_value=runtime), \
                     patch('scripts.task.multi_point.task.MultiPointTask',return_value=task):
                    status=main(['--config',str(path),'--output-dir',temp,'--rviz' if rviz else '--no-rviz'])
                self.assertEqual(status,1 if failed else 0)
                expected=['task']+(['preview'] if rviz and not failed else [])+['runtime_close','client_close']
                self.assertEqual(calls,expected)

    def test_external_preview_replays_until_user_quits_without_closing_external_process(self):
        from unittest.mock import Mock,patch
        from scripts.task.common.runtime import Runtime
        client=Mock()
        client.cancel_requested.wait.return_value=False
        runtime=Runtime(client,'.','.')
        runtime.external_rviz=True
        with patch.object(runtime,'preview_key',side_effect=['','q']):
            runtime.wait_for_rviz()
        client.replay_preview.assert_called_once_with()
        self.assertEqual(runtime.processes,[])
        client.execute.assert_not_called()
        client.stop.assert_not_called()

    def test_existing_stack_and_rviz_are_reused_without_spawn(self):
        from unittest.mock import Mock,patch
        from scripts.task.common.runtime import Runtime
        runtime=Runtime(Mock(),'.','.')
        with patch.object(runtime,'acquire'),patch.object(runtime,'names',return_value={'rviz2','move_group'}), \
             patch.object(runtime,'spawn') as spawn,patch.object(runtime,'start_rviz') as start, \
             patch.object(runtime,'wait_for_display') as ready,patch('scripts.task.common.runtime.time.sleep'):
            runtime.prepare('attach',True)
        self.assertTrue(runtime.external_rviz)
        spawn.assert_not_called();start.assert_not_called();ready.assert_called_once_with()

    def test_wait_for_display_requires_a_stable_subscription(self):
        from unittest.mock import Mock,patch
        from scripts.task.common.runtime import Runtime
        client=Mock();client.cancel_requested.is_set.return_value=False
        client.display.get_subscription_count.side_effect=[0,1,1,1,1]
        runtime=Runtime(client,'.','.')
        now=[0.0]
        def step(_): now[0]+=1.0
        with patch('scripts.task.common.runtime.time.monotonic',side_effect=lambda:now[0]), \
             patch('scripts.task.common.runtime.time.sleep',side_effect=step):
            runtime.wait_for_display(timeout=10)
        self.assertGreaterEqual(now[0],3.0)

    def test_missing_display_times_out_with_actionable_error(self):
        from unittest.mock import Mock,patch
        from scripts.task.common.runtime import Runtime
        client=Mock();client.cancel_requested.is_set.return_value=False
        client.display.get_subscription_count.return_value=0
        runtime=Runtime(client,'.','.')
        now=[0.0]
        def step(_): now[0]+=1.0
        with patch('scripts.task.common.runtime.time.monotonic',side_effect=lambda:now[0]), \
             patch('scripts.task.common.runtime.time.sleep',side_effect=step):
            with self.assertRaisesRegex(RuntimeError,'Trajectory Topic'):
                runtime.wait_for_display(timeout=3)

    def test_preview_replay_only_publishes_cached_messages(self):
        from unittest.mock import Mock
        from scripts.task.common.moveit_client import MoveItClient
        from moveit_msgs.msg import DisplayTrajectory
        from visualization_msgs.msg import MarkerArray
        client=SimpleNamespace(preview_message=DisplayTrajectory(),target_message=MarkerArray(),display=Mock(),targets=Mock())
        MoveItClient.replay_preview(client)
        client.display.publish.assert_called_once_with(client.preview_message)
        client.targets.publish.assert_called_once_with(client.target_message)

    def test_preview_message_preserves_all_segments_and_initial_state(self):
        from unittest.mock import Mock
        from scripts.task.common.moveit_client import MoveItClient
        client=SimpleNamespace(display=Mock())
        first,second=RobotTrajectory(),RobotTrajectory()
        first.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.1]*6)]
        second.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.2]*6)]
        MoveItClient.publish_plan(client,[0.0]*6,[first,second])
        message=client.display.publish.call_args.args[0]
        self.assertEqual(list(message.trajectory_start.joint_state.position),[0.0]*6)
        self.assertEqual(message.trajectory,[first,second])
        self.assertIs(client.preview_message,message)

@unittest.skipUnless(ROS,'Use the ROS runtime wrapper')
class DiagnosticHealthTests(unittest.TestCase):
    def feedback(self):
        import time
        from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
        from scripts.task.common.robot_feedback import Feedback
        from scripts.task.common.schema import JOINTS
        f=Feedback()
        stamp=time.time();sec=int(stamp);nano=int((stamp-sec)*1e9)
        joint=JointState(name=JOINTS,position=[0.]*6)
        joint.header.stamp.sec=sec;joint.header.stamp.nanosec=nano
        f.joint_callback(joint)
        for kind,names in [('motor',JOINTS),('arm',['arm'])]:
            msg=DiagnosticArray();msg.header.stamp=copy.deepcopy(joint.header.stamp)
            msg.status=[DiagnosticStatus(name=name,level=DiagnosticStatus.OK,message='enabled',values=[KeyValue(key='enabled',value='true')]) for name in names]
            f.diagnostic_callback(kind,msg)
        return f

    def test_real_ros_byte_ok_is_accepted(self):
        self.feedback().health(1)

    def test_fault_warn_stale_and_disabled_remain_blocked(self):
        from diagnostic_msgs.msg import DiagnosticStatus
        for level,label in [(DiagnosticStatus.ERROR,'ERROR'),(DiagnosticStatus.WARN,'WARN'),(DiagnosticStatus.STALE,'STALE')]:
            f=self.feedback();s=f.diagnostics['motor'][1].status[0]
            s.level=level;s.message='test fault detail'
            with self.assertRaisesRegex(RuntimeError,label+': test fault detail'): f.health(1)
        f=self.feedback();f.diagnostics['motor'][1].status[0].values[0].value='false'
        with self.assertRaisesRegex(RuntimeError,'disabled'): f.health(1)
        f=self.feedback();f.diagnostics['arm'][1].status[0].level=DiagnosticStatus.ERROR
        with self.assertRaisesRegex(RuntimeError,'arm/arm: ERROR'): f.health(1)

    def test_slower_diagnostics_have_independent_freshness_window(self):
        f=self.feedback()
        for kind in ('motor','arm'):
            received,msg=f.diagnostics[kind]
            msg.header.stamp.sec-=1
            f.diagnostics[kind]=(received-1,msg)
        with self.assertRaisesRegex(RuntimeError,'diagnostics missing/stale'): f.health(.5)
        f.health(.5,1.5)

    def test_independent_window_still_rejects_expired_source_receive_and_joint(self):
        for field in ('source','receive','joint','future'):
            with self.subTest(field=field):
                f=self.feedback()
                if field=='source': f.diagnostics['arm'][1].header.stamp.sec-=2
                if field=='future': f.diagnostics['arm'][1].header.stamp.sec+=1
                if field=='receive':
                    received,msg=f.diagnostics['motor'];f.diagnostics['motor']=(received-2,msg)
                if field=='joint':
                    from dataclasses import replace
                    f.sample=replace(f.sample,stamp=f.sample.stamp-1,received=f.sample.received-1)
                with self.assertRaises(RuntimeError): f.health(.5,1.5)

    def test_longer_window_never_accepts_faults_or_disabled_motor(self):
        from diagnostic_msgs.msg import DiagnosticStatus
        for fault in (False,True):
            with self.subTest(fault=fault):
                f=self.feedback()
                f.diagnostics['motor'][1].header.stamp.sec-=1
                status=f.diagnostics['motor'][1].status[0]
                if fault: status.level=DiagnosticStatus.ERROR
                else: status.values[0].value='false'
                with self.assertRaises(RuntimeError): f.health(.5,1.5)

    def test_schema_and_client_use_separate_timeouts(self):
        from unittest.mock import Mock
        from scripts.task.common.schema import validate_config
        from scripts.task.common.moveit_client import MoveItClient
        config={'points':[dict(name='a',type='joint',joints_rad=[0.]*6)]}
        settings=validate_config(config)['settings']
        self.assertEqual(settings['feedback_timeout'],.5)
        self.assertEqual(settings['diagnostic_timeout'],1.5)
        client=SimpleNamespace(settings=settings,feedback=Mock(),cancel_requested=Mock())
        client.cancel_requested.is_set.return_value=False
        MoveItClient.tick(client,True)
        client.feedback.health.assert_called_once_with(.5,1.5)
        for value in (0,-1,float('nan')):
            config['settings']={'diagnostic_timeout':value}
            with self.assertRaises(ValueError): validate_config(config)

    def test_missing_and_expired_diagnostics_remain_blocked(self):
        f=self.feedback();f.diagnostics['motor'][1].status.pop(0)
        with self.assertRaisesRegex(RuntimeError,'joint1: status missing'): f.health(1)
        f=self.feedback();received,msg=f.diagnostics['motor'];f.diagnostics['motor']=(received-2,msg)
        with self.assertRaisesRegex(RuntimeError,'motor diagnostics missing/stale'): f.health(1)
        f=self.feedback();f.diagnostics['arm'][1].header.stamp.sec-=2
        with self.assertRaisesRegex(RuntimeError,'source timestamp stale'): f.health(1)

@unittest.skipUnless(ROS,'Use the ROS runtime wrapper')
class ReviewedExecutionTests(unittest.TestCase):
    client=TaskMessageTests.client
    config=TaskMessageTests.config
    def prepare(self,temp):
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        client=self.client();config=self.config(True);scene={'enabled':False,'objects':[]}
        preview=MultiPointTask(client,config,scene,RunLog(temp))
        preview.run()
        return client,config,scene,preview.reviewed_plan

    def test_reviewed_execution_reuses_exact_trajectories_without_planning_again(self):
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        with tempfile.TemporaryDirectory() as temp:
            client,config,scene,reviewed=self.prepare(temp)
            planned=len(client.starts)
            result=MultiPointTask(client,config,scene,RunLog(temp),True).run(reviewed=reviewed)
            self.assertTrue(result['reviewed_execution'])
            self.assertEqual(len(client.starts),planned)
            self.assertEqual(client.executions,3)
            self.assertEqual(client.sent,[part['trajectory'] for part in reviewed['segments']])

    def test_changed_start_scene_or_model_reject_before_execution(self):
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        from scripts.task.common.planning_scene import obstacle_message
        for change in ('start','scene','model'):
            with self.subTest(change=change),tempfile.TemporaryDirectory() as temp:
                client,config,scene,reviewed=self.prepare(temp)
                if change=='start': client.q=(0.5,)*6
                if change=='model': client.model=lambda:['different model',client.srdf]
                if change=='scene':
                    altered=Scene()
                    altered.world.collision_objects=[obstacle_message(dict(id='new',shape='box',dimensions_m=[0.1]*3,position_m=[1.,0.,0.],orientation_xyzw=[0.,0.,0.,1.],frame_id='base_link'))]
                    client.scene.get=lambda:altered
                with self.assertRaisesRegex(RuntimeError,'changed since preview'):
                    MultiPointTask(client,config,scene,RunLog(temp),True).run(reviewed=reviewed)
                self.assertEqual(client.executions,0)

    def test_fault_before_reviewed_execution_blocks_motion(self):
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        with tempfile.TemporaryDirectory() as temp:
            client,config,scene,reviewed=self.prepare(temp)
            def stable(*args,**kwargs): raise RuntimeError('motor/joint1: ERROR: collision_tripped')
            client.stable=stable
            with self.assertRaisesRegex(RuntimeError,'collision_tripped'):
                MultiPointTask(client,config,scene,RunLog(temp),True).run(reviewed=reviewed)
            self.assertEqual(client.executions,0)

@unittest.skipUnless(ROS,'Use the ROS runtime wrapper')
class InteractiveKeyTests(unittest.TestCase):
    def runtime(self):
        from unittest.mock import Mock
        from scripts.task.common.runtime import Runtime
        client=Mock();client.cancel_requested.wait.return_value=False
        runtime=Runtime(client,'.','.');runtime.external_rviz=True
        return runtime

    def test_execution_requires_explicit_e_and_is_once_only(self):
        from unittest.mock import Mock
        runtime=self.runtime();execute=Mock()
        with patch.object(runtime,'preview_key',side_effect=['','e','e','q']):
            runtime.wait_for_rviz(on_execute=execute)
        execute.assert_called_once_with()
        runtime.client.replay_preview.assert_called_once_with()

    def test_plain_preview_cannot_execute(self):
        runtime=self.runtime()
        with patch.object(runtime,'preview_key',side_effect=['e','q']): runtime.wait_for_rviz()
        runtime.client.execute.assert_not_called()

    def test_entry_routes_interactive_mode_to_persistent_session_without_executing(self):
        from unittest.mock import Mock
        from pathlib import Path
        from scripts.task.multi_point.run import main
        with tempfile.TemporaryDirectory() as temp, \
             patch('scripts.task.common.moveit_client.MoveItClient',return_value=Mock()), \
             patch('scripts.task.common.runtime.Runtime',return_value=Mock()), \
             patch('scripts.task.multi_point.console.run_session') as session, \
             patch('scripts.task.multi_point.task.MultiPointTask') as task:
            result=main(['--output-dir',temp,'--rviz','--interactive'])
        self.assertEqual(result,0)
        task.assert_not_called()
        session.assert_called_once()
        self.assertIsNone(session.call_args.args[3].config)

    def test_cli_rejects_incompatible_modes_and_missing_rviz(self):
        from scripts.task.multi_point.run import main
        for arguments in (['--interactive'],['--rviz','--interactive','--execute']):
            with self.subTest(arguments=arguments),self.assertRaises(SystemExit) as error:
                main(arguments)
            self.assertEqual(error.exception.code,2)

    def test_motion_interrupt_propagates_as_failure(self):
        from unittest.mock import Mock
        runtime=self.runtime();execute=Mock(side_effect=KeyboardInterrupt)
        with patch.object(runtime,'preview_key',return_value='e'):
            with self.assertRaises(InterruptedError): runtime.wait_for_rviz(on_execute=execute)
        execute.assert_called_once_with()

if __name__=='__main__': unittest.main()
