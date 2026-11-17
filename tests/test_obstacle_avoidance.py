"""Virtual scene isolation, spline inspection and comparison failure paths."""
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

try:
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PlanningScene as Scene, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from scripts.task.common.planning_scene import PlanningScene, obstacle_message, fingerprint
    from scripts.task.common.schema import JOINTS, load_config, load_scene
    from scripts.task.obstacle_avoidance.client import PlanningClient
    from scripts.task.obstacle_avoidance.task import ObstacleTask, load_inputs, with_margin
    from scripts.task.obstacle_avoidance.validation import interpolate, samples, inspect
    ROS = True
except ImportError:
    ROS = False

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT/'scripts/task/obstacle_avoidance/config/config.yaml'
CONFIG = DEFAULT_CONFIG.with_name('config_simple.yaml')
SCENE = CONFIG.with_name('scene_simple.yaml')


@unittest.skipUnless(ROS, 'Use ROS runtime wrapper')
class ObstacleTests(unittest.TestCase):
    def trajectory(self):
        msg = RobotTrajectory()
        msg.joint_trajectory.joint_names = list(JOINTS)
        a = JointTrajectoryPoint(positions=[0.]*6, velocities=[1.]*6, accelerations=[0.]*6)
        b = JointTrajectoryPoint(positions=[1.]*6, velocities=[1.]*6, accelerations=[0.]*6)
        b.time_from_start.sec = 1
        msg.joint_trajectory.points = [a,b]
        return msg

    def test_default_inputs_and_no_hardware_execution_interface(self):
        config, scene = load_inputs(DEFAULT_CONFIG, DEFAULT_CONFIG.with_name('scene.yaml'))
        self.assertEqual(len(config['points']), 4)
        self.assertEqual(len(scene['objects']), 3)
        simple, scene = load_inputs(CONFIG, SCENE)
        self.assertEqual(len(simple['points']), 2)
        self.assertEqual(len(scene['objects']), 1)
        self.assertFalse(hasattr(PlanningClient, 'execute'))
        self.assertFalse(hasattr(PlanningClient, 'restore_task_control'))

    def test_quintic_and_cubic_interpolation_and_joint_order(self):
        msg = self.trajectory()
        a,b = msg.joint_trajectory.points
        self.assertEqual(interpolate(a,b,.25,1.), [.25]*6)
        a.accelerations = []; b.accelerations = []
        self.assertEqual(interpolate(a,b,.25,1.), [.25]*6)
        a.velocities = []; b.velocities = []
        a.positions = list(map(float,range(6)))
        b.positions = [q+1 for q in a.positions]
        msg.joint_trajectory.joint_names = list(reversed(JOINTS))
        values = list(samples(msg))
        self.assertEqual(values[0][1], [5.,4.,3.,2.,1.,0.])
        self.assertEqual(values[-1][1], [6.,5.,4.,3.,2.,1.])
        self.assertLessEqual(max(b[0]-a[0] for a,b in zip(values,values[1:])), .05+1e-12)

    def test_inspection_detects_between_waypoint_collision(self):
        c = SimpleNamespace(tick=Mock(),state_validity=Mock(side_effect=lambda q:
            SimpleNamespace(valid=not .45 < q[0] < .55,contacts=[])))
        report = inspect(c,self.trajectory(),stop_on_collision=True)
        self.assertFalse(report['valid'])
        self.assertGreater(report['checked_samples'], 2)
        self.assertFalse(report['continuous_collision_proof'])

    def test_position_endpoint_checks_do_not_hide_spline_overshoot(self):
        msg = self.trajectory(); a,b = msg.joint_trajectory.points
        a.positions = b.positions = [0.]*6
        a.velocities = [4.]*6; b.velocities = [-4.]*6
        values = list(samples(msg))
        self.assertGreater(max(q[0] for _,q in values), .5)

    def test_clearance_changes_geometry_without_mutating_visible_scene(self):
        scene = load_scene(SCENE,'base_link'); saved = deepcopy(scene)
        expanded = with_margin(scene,.025)
        self.assertEqual(scene,saved)
        for actual, expected in zip(expanded['objects'][0]['dimensions_m'], [.15,.15,.25]):
            self.assertAlmostEqual(actual,expected)

    def test_scene_prefix_preserves_unrelated_objects(self):
        config = load_scene(SCENE,'base_link')
        old = obstacle_message(dict(config['objects'][0],id='old'),'obstacle_avoidance/')
        other = obstacle_message(config['objects'][0])
        state = Scene(); state.world.collision_objects = [old, other]
        def call(client,request,**kwargs):
            if client == 'get': return SimpleNamespace(scene=deepcopy(state))
            objects = {obj.id:obj for obj in state.world.collision_objects}
            for obj in request.scene.world.collision_objects:
                if obj.operation == obj.REMOVE: objects.pop(obj.id,None)
                else: objects[obj.id] = obj
            state.world.collision_objects = list(objects.values())
            return SimpleNamespace(success=True)
        manager = PlanningScene.__new__(PlanningScene)
        manager.context = SimpleNamespace(call=call)
        manager.prefix = 'obstacle_avoidance/'
        manager.getter, manager.setter = 'get','set'
        manager.apply(config)
        self.assertEqual({o.id for o in state.world.collision_objects}, {'multi_point/box','obstacle_avoidance/box'})
        manager.apply(dict(enabled=True,objects=[]))
        self.assertEqual([o.id for o in state.world.collision_objects], ['multi_point/box'])

    def fixture(self, directory):
        config, scene = load_inputs(CONFIG,SCENE)
        state = Scene()
        def apply(value):
            state.world.collision_objects = [obstacle_message(o,'obstacle_avoidance/') for o in value['objects']]
            return deepcopy(state)
        c = SimpleNamespace(settings=config['settings'], start=config['points'][0]['joints_rad'],
            cancel_requested=threading.Event(), tick=Mock(), connect_model=Mock(),check_joints=Mock(),
            model_sha256='model',urdf='urdf',srdf='srdf',clear_preview=Mock(),show=Mock(),
            scene=SimpleNamespace(apply=Mock(side_effect=apply),get=Mock(side_effect=lambda:deepcopy(state))),
            state_validity=Mock(return_value=SimpleNamespace(valid=True)),
            plan=Mock(return_value=(self.trajectory(),config['points'][1]['joints_rad'],1.)),
            fk=Mock(return_value=dict(position_m=[.2,0.,.4],orientation_xyzw=[0.,0.,0.,1.])))
        trajectory = self.trajectory()
        trajectory.joint_trajectory.points[0].positions = config['points'][0]['joints_rad']
        trajectory.joint_trajectory.points[-1].positions = config['points'][1]['joints_rad']
        c.plan.return_value = trajectory, config['points'][1]['joints_rad'], 1.
        return ObstacleTask(c,CONFIG,SCENE,Path(directory)),c

    def test_comparison_requires_blocked_baseline_and_valid_avoidance(self):
        for statuses in ((True,False,True),(True,True),(True,False,False)):
            with self.subTest(statuses=statuses), tempfile.TemporaryDirectory() as temp:
                task,c = self.fixture(temp)
                with patch('scripts.task.obstacle_avoidance.task.inspect',
                           side_effect=[dict(valid=v,checked_samples=20) for v in statuses]):
                    if statuses == (True,False,True):
                        result = task.compare(); self.assertTrue(result['success'])
                        c.show.assert_called_once()
                        self.assertEqual(c.plan.call_count,2)
                        for call in c.plan.call_args_list: self.assertFalse(call.kwargs['publish'])
                        c.scene.get.return_value = None
                        c.scene.get.side_effect = lambda:Scene()
                        with self.assertRaisesRegex(RuntimeError,'场景已变化'): task.show()
                    else:
                        with self.assertRaises(RuntimeError): task.compare()
                        c.show.assert_not_called()
                    report = json.loads(next(Path(temp).glob('rounds/*/result.json')).read_text())
                    self.assertEqual(report['success'], statuses == (True,False,True))
                    self.assertEqual(c.scene.apply.call_args.args[0],load_scene(SCENE,'base_link'))

    def test_invalid_goal_test_restores_original_scene_on_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            task,c = self.fixture(temp); task.reload()
            with self.assertRaisesRegex(RuntimeError,'验证未成立'):
                task.check_invalid_goal(task.config['points'][1]['joints_rad'], [.2,0.,.4])
            self.assertEqual(c.scene.apply.call_args.args[0],task.scene)

    def multi_fixture(self, directory):
        task, c = self.fixture(directory)
        task.config_path = DEFAULT_CONFIG
        task.scene_path = DEFAULT_CONFIG.with_name('scene.yaml')
        def plan(point, start, **kwargs):
            trajectory = self.trajectory()
            endpoint = list(point['joints_rad'])
            endpoint[0] += .001  # A valid endpoint need not equal the nominal target.
            trajectory.joint_trajectory.points[0].positions = list(start)
            trajectory.joint_trajectory.points[-1].positions = endpoint
            return trajectory, endpoint, 1.
        c.plan.side_effect = plan
        return task, c

    def test_multisegment_comparison_chains_actual_endpoints_and_replays_both(self):
        with tempfile.TemporaryDirectory() as temp:
            task, c = self.multi_fixture(temp)
            statuses = [True]*3 + [False]*3 + [True]*3
            with patch('scripts.task.obstacle_avoidance.task.inspect',
                       side_effect=[dict(valid=v, checked_samples=20) for v in statuses]):
                result = task.compare()
            self.assertTrue(result['success'])
            self.assertEqual(result['segment_count'], 3)
            self.assertEqual(c.plan.call_count, 6)
            for base in (0, 3):
                for offset in (1, 2):
                    previous = c.plan.call_args_list[base+offset-1].args[0]['joints_rad']
                    actual_start = c.plan.call_args_list[base+offset].args[1]
                    self.assertAlmostEqual(actual_start[0], previous[0]+.001)
            task.show(True)
            self.assertEqual(len(c.show.call_args.args[0]), 3)
            self.assertTrue(c.show.call_args.kwargs['baseline'])
            task.show(False)
            self.assertFalse(c.show.call_args.kwargs['baseline'])
            for stage in ('baseline', 'avoidance'):
                saved = json.loads(next(Path(temp).glob(f'rounds/*/{stage}_trajectory.json')).read_text())
                self.assertEqual(len(saved['trajectories']), 3)
                self.assertEqual(saved['point_names'], ['A', 'B', 'C', 'D'])

    def test_later_segment_collision_invalidates_whole_preview(self):
        with tempfile.TemporaryDirectory() as temp:
            task, c = self.multi_fixture(temp)
            statuses = [True]*3 + [False]*3 + [True, True, False]
            with patch('scripts.task.obstacle_avoidance.task.inspect',
                       side_effect=[dict(valid=v, checked_samples=20) for v in statuses]):
                with self.assertRaisesRegex(RuntimeError, '复查发现碰撞'): task.compare()
            self.assertIsNone(task.baseline)
            self.assertIsNone(task.avoidance)
            c.show.assert_not_called()
            self.assertEqual(c.scene.apply.call_args.args[0], task.scene)

    def test_sequence_rejects_discontinuity_and_intermediate_goal_error(self):
        for field, message in (('start', '起点不连续'), ('end', '终点超出')):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as temp:
                task, c = self.multi_fixture(temp)
                task.reload()
                original = c.plan.side_effect
                def broken(point, start, **kwargs):
                    trajectory, endpoint, duration = original(point, start, **kwargs)
                    if point['name'] == 'C':
                        if field == 'start': trajectory.joint_trajectory.points[0].positions[0] += .1
                        else: endpoint[0] += .1
                    return trajectory, endpoint, duration
                c.plan.side_effect = broken
                with self.assertRaisesRegex(RuntimeError, f'第 2 段 B → C: .*{message}'):
                    task.plan_sequence(Mock(), 'avoidance')
                c.show.assert_not_called()

    def test_all_waypoint_labels_have_unique_marker_ids_and_old_show_still_works(self):
        c = SimpleNamespace(start=[0.]*6, settings={'frame_id': 'base_link'},
                            waypoint_names=['A','B','C','D'], publish_plan=Mock(), markers=Mock())
        trajectories = [self.trajectory() for _ in range(3)]
        points = [[.2, float(i)*.1, .4] for i in range(4)]
        PlanningClient.show(c, trajectories, points, points)
        self.assertEqual(c.publish_plan.call_args.args[1], trajectories)
        markers = c.markers.publish.call_args.args[0].markers[1:]
        self.assertEqual(len({(m.ns,m.id) for m in markers}), len(markers))
        self.assertEqual([m.text for m in markers if m.text], ['A','B','C','D','AVOIDANCE'])
        PlanningClient.show(c, trajectories[0], points[:2], points[:2], baseline=True)
        self.assertEqual(c.publish_plan.call_args.args[1], trajectories[:1])

    def test_control_planning_uses_requested_real_start_and_refuses_collision(self):
        from scripts.task.common.moveit_client import MoveItClient
        from scripts.task.obstacle_avoidance.control_client import ObstacleControlClient
        trajectory = self.trajectory()
        c = SimpleNamespace(scene=SimpleNamespace(get=Mock(return_value=Scene())), publish_plan=Mock())
        point = dict(name='A', type='joint', joints_rad=[1.]*6)
        for valid in (False, True):
            with self.subTest(valid=valid), \
                 patch.object(MoveItClient, 'plan', return_value=(trajectory, [1.]*6, 1.)) as plan, \
                 patch('scripts.task.obstacle_avoidance.control_client.inspect', return_value={'valid':valid}):
                # Keep the real subclass for super(), without constructing ROS objects.
                client = ObstacleControlClient.__new__(ObstacleControlClient)
                client.tick = Mock()
                client.scene, client.publish_plan = c.scene, c.publish_plan
                if valid:
                    client.plan(point, [.2]*6)
                    c.publish_plan.assert_called_once_with([.2]*6, [trajectory])
                else:
                    with self.assertRaisesRegex(RuntimeError, '仍有轨迹采样碰撞'):
                        client.plan(point, [.2]*6)
                    c.publish_plan.assert_not_called()
                self.assertEqual(plan.call_count,1 if valid else 5)
                plan.assert_called_with(point, [.2]*6, publish=False)

    def test_control_retries_only_retryable_failures_and_publishes_only_checked_path(self):
        from scripts.task.common.moveit_client import MoveItClient, PlanningError
        from scripts.task.obstacle_avoidance.control_client import ObstacleControlClient
        client=ObstacleControlClient.__new__(ObstacleControlClient)
        client.tick=Mock();client.scene=SimpleNamespace(get=Mock(return_value=Scene()));client.publish_plan=Mock()
        trajectory=self.trajectory();result=(trajectory,[1.]*6,1.)
        point=dict(name='C',type='joint',joints_rad=[1.]*6)
        failure=PlanningError(SimpleNamespace(val=99999))
        with patch.object(MoveItClient,'plan',side_effect=[failure,result,result]) as plan, \
             patch('scripts.task.obstacle_avoidance.control_client.inspect',side_effect=[{'valid':False,'collisions':[{'time':.4}]},{'valid':True}]) as inspect:
            self.assertEqual(client.plan(point,[.2]*6),result)
            self.assertEqual(plan.call_count,3)
            self.assertEqual(inspect.call_count,2)
            self.assertEqual(len(client.planning_diagnostics),3)
            client.publish_plan.assert_called_once_with([.2]*6,[trajectory])
        client.publish_plan.reset_mock()
        for error in (PlanningError(SimpleNamespace(val=-10)),InterruptedError('cancel'),ValueError('limits')):
            with patch.object(MoveItClient,'plan',side_effect=error) as plan:
                with self.assertRaises(type(error)):client.plan(point,[.2]*6)
                plan.assert_called_once()
                client.publish_plan.assert_not_called()
        # Fingerprint uses scene geometry, not the scene label.
        with patch('scripts.task.obstacle_avoidance.control_client.fingerprint',side_effect=['a','a','b']), \
             patch.object(MoveItClient,'plan',return_value=result), \
             patch('scripts.task.obstacle_avoidance.control_client.inspect',return_value={'valid':True}):
            with self.assertRaisesRegex(RuntimeError,'场景变化'):client.plan(point,[.2]*6)
            client.publish_plan.assert_not_called()

    def test_control_planner_config_is_named_and_verified(self):
        from scripts.task.obstacle_avoidance.control_client import ObstacleControlClient
        from moveit_msgs.srv import SetPlannerParams,GetPlannerParams
        client=ObstacleControlClient.__new__(ObstacleControlClient)
        client.settings={'group':'arm'};client.node=Mock();client.tick=Mock();requests=[]
        def call(service,request,**kwargs):
            requests.append(request)
            if isinstance(request,GetPlannerParams.Request):return SimpleNamespace(params=requests[0].params)
            return SimpleNamespace()
        client.call=Mock(side_effect=call)
        client.configure_planner()
        self.assertEqual(len(requests),2)
        self.assertIsInstance(requests[0],SetPlannerParams.Request)
        self.assertEqual(requests[0].planner_config,'ObstacleControl')
        self.assertEqual(dict(zip(requests[0].params.keys,requests[0].params.values))['longest_valid_segment_fraction'],'0.0001')
        self.assertEqual(client.node.destroy_client.call_count,2)

    def test_control_entry_reuses_interactive_panel_without_auto_execution(self):
        from scripts.task.obstacle_avoidance.control import main
        from scripts.task.obstacle_avoidance.control_client import ObstacleControlClient
        with patch('scripts.task.multi_point.run.main', return_value=0) as panel, \
             patch.dict('os.environ', {}, clear=False):
            self.assertEqual(main(['--domain','96','--stack','attach']), 0)
        args = panel.call_args.args[0]
        self.assertIn('--interactive', args)
        self.assertIn('--rviz', args)
        self.assertNotIn('--execute', args)
        self.assertIn(str(DEFAULT_CONFIG), args)
        self.assertIn(str(DEFAULT_CONFIG.with_name('scene.yaml')), args)
        self.assertIs(panel.call_args.kwargs['client_type'], ObstacleControlClient)
        with patch('scripts.task.multi_point.run.main') as panel:
            with self.assertRaises(SystemExit): main(['--execute'])
            panel.assert_not_called()

    def test_interactive_entry_routes_to_same_window_panel_without_preview_runtime(self):
        from scripts.task.obstacle_avoidance.run import main
        with patch('scripts.task.obstacle_avoidance.control.main', return_value=0) as control, \
             patch('scripts.task.obstacle_avoidance.runtime.PreviewRuntime') as runtime:
            self.assertEqual(main(['--interactive', '--control-stack', 'attach']), 0)
        runtime.assert_not_called()
        self.assertIn('--stack', control.call_args.args[0])
        self.assertIn('attach', control.call_args.args[0])


    def test_runtime_refuses_occupied_domain_before_starting_anything(self):
        from scripts.task.obstacle_avoidance.runtime import PreviewRuntime
        client = Mock(); client.node.get_name.return_value='obstacle_preview_test'
        runtime = PreviewRuntime(client,ROOT,Path('/tmp'))
        with patch.object(runtime,'acquire'), patch.object(runtime,'spawn') as spawn, \
             patch.object(runtime,'names',return_value={'obstacle_preview_test','agx_arm_ctrl_single_node'}), \
             patch('scripts.task.obstacle_avoidance.runtime.time.sleep'):
            with self.assertRaisesRegex(RuntimeError,'空闲 ROS 域'): runtime.prepare(False)
        spawn.assert_not_called()
        client.connect_model.assert_not_called()

    def test_planning_retry_is_bounded_and_does_not_relax_request(self):
        from scripts.task.common.moveit_client import PlanningError
        from moveit_msgs.msg import MoveItErrorCodes
        with tempfile.TemporaryDirectory() as temp:
            task,c = self.fixture(temp); task.reload()
            error = PlanningError(MoveItErrorCodes(val=99999))
            expected = c.plan.return_value
            c.plan.side_effect = [error,error,expected]
            self.assertEqual(task.plan(Mock(),c.start,'avoidance'),expected)
            self.assertEqual(c.plan.call_count,3)
            self.assertEqual(c.plan.call_args_list[0],c.plan.call_args_list[-1])
            c.plan.reset_mock(); c.plan.side_effect=error
            with self.assertRaises(PlanningError): task.plan(Mock(),c.start,'avoidance')
            self.assertEqual(c.plan.call_count,5)

    def test_named_planner_is_configured_and_selected(self):
        from scripts.task.obstacle_avoidance.runtime import PreviewRuntime
        from scripts.task.common.moveit_client import MoveItClient
        from scripts.task.common.schema import DEFAULTS
        from moveit_msgs.srv import SetPlannerParams, GetPlannerParams
        from rcl_interfaces.srv import SetParametersAtomically
        client = Mock(); client.node.get_name.return_value='preview_test'
        client.settings=dict(DEFAULTS); client.planner_id=PlanningClient.planner_id
        saved=[]
        def call(service,request,**kwargs):
            if isinstance(request,SetParametersAtomically.Request):
                return SimpleNamespace(result=SimpleNamespace(successful=True))
            if isinstance(request,SetPlannerParams.Request):
                saved.append(request);return SimpleNamespace()
            if isinstance(request,GetPlannerParams.Request):
                self.assertEqual(request.planner_config,saved[0].planner_config)
                return SimpleNamespace(params=saved[0].params)
            return SimpleNamespace(values=[SimpleNamespace(type=1,bool_value=False)])
        client.call.side_effect=call
        runtime=PreviewRuntime(client,ROOT,Path('/tmp'))
        with patch.object(runtime,'acquire'),patch.object(runtime,'spawn') as spawn, \
             patch.object(runtime,'names',return_value={'preview_test'}), \
             patch('scripts.task.obstacle_avoidance.runtime.time.sleep'),patch.dict('os.environ',ROS_DOMAIN_ID='87'):
            runtime.prepare(False)
        self.assertEqual(saved[0].planner_config,'ObstacleDemo')
        self.assertEqual(saved[0].group,'arm')
        self.assertEqual(spawn.call_count,2)
        self.assertIn('allow_trajectory_execution:=false',spawn.call_args_list[0].args[1])
        msg=self.trajectory()
        c=SimpleNamespace(settings=dict(DEFAULTS),planner_id=PlanningClient.planner_id,
            check_state=Mock(),check_joints=Mock(),tick=Mock(),planning=object(),
            call=Mock(return_value=SimpleNamespace(motion_plan_response=SimpleNamespace(
                error_code=SimpleNamespace(val=1),trajectory=msg))))
        MoveItClient.plan(c,dict(type='joint',joints_rad=[1.]*6),[0.]*6,publish=False)
        self.assertEqual(c.call.call_args.args[1].motion_plan_request.planner_id,saved[0].planner_config)

    def test_cli_config_check_needs_no_ros_runtime(self):
        import subprocess
        result=subprocess.run(['bash','scripts/piperl/run.sh','--python-only','python',
            'scripts/task/obstacle_avoidance/run.py','--check-config'],
            cwd=ROOT,capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('尚未验证可达性',result.stdout)


if __name__ == '__main__': unittest.main()
