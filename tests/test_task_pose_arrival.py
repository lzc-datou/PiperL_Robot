"""Planning tolerance and measured TCP arrival; no ROS nodes or robot commands."""
import math
from types import SimpleNamespace
from unittest.mock import Mock,patch
import unittest
try:
    from scripts.task.common.moveit_client import MoveItClient
    from scripts.task.common.schema import DEFAULTS,JOINTS,validate_config
    from scripts.task.common.robot_feedback import Sample
    ROS=True
except ImportError:
    ROS=False


@unittest.skipUnless(ROS,'Use ROS runtime wrapper')
class PoseArrivalTests(unittest.TestCase):
    def point(self):
        return dict(name='point_002_pose',type='pose',frame_id='base_link',
                    position_m=[0.,0.,0.],orientation_xyzw=[0.,0.,0.,1.])

    def test_pose_planner_reserves_half_arrival_tolerance_without_changing_acceptance(self):
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        trajectory=RobotTrajectory()
        trajectory.joint_trajectory.joint_names=JOINTS
        a=JointTrajectoryPoint(positions=[0.]*6)
        b=JointTrajectoryPoint(positions=[0.]*6);b.time_from_start.sec=1
        trajectory.joint_trajectory.points=[a,b]
        c=SimpleNamespace(settings=dict(DEFAULTS),check_state=Mock(),check_joints=Mock(),
            planning=object(),tick=Mock(),publish_plan=Mock(),
            call=Mock(return_value=SimpleNamespace(motion_plan_response=SimpleNamespace(
                error_code=SimpleNamespace(val=1),trajectory=trajectory))))
        MoveItClient.plan(c,self.point(),[0.]*6,publish=False)
        goal=c.call.call_args.args[1].motion_plan_request.goal_constraints[0]
        self.assertEqual(goal.position_constraints[0].constraint_region.primitives[0].dimensions[0],.0015)
        self.assertAlmostEqual(goal.orientation_constraints[0].absolute_x_axis_tolerance,.02/(2*math.sqrt(3)))
        self.assertEqual(c.settings['position_tolerance_m'],.003)
        self.assertEqual(c.settings['orientation_tolerance_rad'],.02)
        c.publish_plan.assert_not_called()

    def test_both_task_planners_receive_equivalent_pose_formats(self):
        from moveit_msgs.msg import PlanningScene, RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from scripts.task.obstacle_avoidance.control_client import ObstacleControlClient

        position = [.2, -.1, .3]
        quaternion = [.5, .5, -.5, .5]
        points = validate_config({'points': [
            dict(name='quat', type='pose', position_m=position, orientation_xyzw=quaternion),
            dict(name='rpy', type='pose', position_m=position, rpy=[90, 90, 0]),
            dict(name='both', type='pose', position_m=position, orientation_xyzw=quaternion, rpy=[90.0001, 90, 0]),
        ]})['points']
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = JOINTS
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.] * 6)]
        for client_type in (MoveItClient, ObstacleControlClient):
            with self.subTest(client=client_type.__name__):
                client = client_type.__new__(client_type)
                client.settings = dict(DEFAULTS)
                client.tick = Mock()
                client.check_state = Mock()
                client.check_joints = Mock()
                client.scene = SimpleNamespace(get=Mock(return_value=PlanningScene()))
                client.planning = object()
                client.call = Mock(return_value=SimpleNamespace(motion_plan_response=SimpleNamespace(
                    error_code=SimpleNamespace(val=1), trajectory=trajectory)))
                with patch('scripts.task.obstacle_avoidance.control_client.inspect', return_value={'valid': True}):
                    for point in points:
                        client.plan(point, [0.] * 6, publish=False)
                goals = [call.args[1].motion_plan_request.goal_constraints[0] for call in client.call.call_args_list]
                for goal in goals:
                    self.assertEqual(goal.position_constraints, goals[0].position_constraints)
                    orientation = goal.orientation_constraints[0]
                    self.assertEqual(orientation.link_name, 'tcp_link')
                    self.assertEqual(orientation.header.frame_id, 'base_link')
                    for field, expected in zip('xyzw', quaternion):
                        self.assertAlmostEqual(getattr(orientation.orientation, field), expected, places=14)

    def fixture(self,errors,angles=None):
        clock=[0.0];seen=[]
        c=SimpleNamespace(settings=dict(DEFAULTS,arrival_timeout=.8,settle_seconds=.15),tick=Mock())
        def current():
            return Sample((0.,)*6,clock[0]+1,clock[0])
        def fk(*args):
            i=len(seen);distance=errors[min(i,len(errors)-1)]
            angle=(angles or [0.])[min(i,len(angles or [0.])-1)]
            seen.append((clock[0],distance,angle))
            return dict(position_m=[distance,0.,0.],orientation_xyzw=[0.,0.,math.sin(angle/2),math.cos(angle/2)])
        c.current=current;c.fk=Mock(side_effect=fk)
        def sleep(seconds): clock[0]+=seconds
        return c,clock,seen,sleep

    def test_malformed_trajectory_derivatives_rejected_before_preview(self):
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        for field in ('velocities','accelerations','effort'):
            for values in ([0.], [float('nan')]*6):
                with self.subTest(field=field,values=values):
                    trajectory=RobotTrajectory()
                    trajectory.joint_trajectory.joint_names=JOINTS
                    point=JointTrajectoryPoint(positions=[0.]*6)
                    setattr(point,field,values)
                    trajectory.joint_trajectory.points=[point]
                    c=SimpleNamespace(settings=dict(DEFAULTS),check_state=Mock(),check_joints=Mock(),
                        planning=object(),tick=Mock(),publish_plan=Mock(),
                        call=Mock(return_value=SimpleNamespace(motion_plan_response=SimpleNamespace(
                            error_code=SimpleNamespace(val=1),trajectory=trajectory))))
                    with self.assertRaises(RuntimeError):MoveItClient.plan(c,self.point(),[0.]*6)
                    c.publish_plan.assert_not_called()

    def run_arrival(self,c,clock,sleep):
        with patch('scripts.task.common.moveit_client.time.monotonic',side_effect=lambda:clock[0]), \
             patch('scripts.task.common.moveit_client.time.sleep',side_effect=sleep):
            return MoveItClient.stable_pose(c,self.point(),[0.]*6)

    def test_transient_3_158mm_error_waits_for_stable_arrival(self):
        c,clock,seen,sleep=self.fixture([.003157634,.0028])
        actual,pose=self.run_arrival(c,clock,sleep)
        self.assertGreaterEqual(clock[0],.17)
        self.assertEqual(pose['position_m'][0],.0028)
        self.assertGreater(len(seen),3)

    def test_sustained_error_still_fails_at_original_3mm_limit(self):
        c,clock,seen,sleep=self.fixture([.003006554])
        with self.assertRaisesRegex(TimeoutError,'point_002_pose.*3.007 mm.*3 mm'):
            self.run_arrival(c,clock,sleep)
        self.assertGreaterEqual(clock[0],.8)

    def test_pose_excursion_resets_stable_window(self):
        c,clock,seen,sleep=self.fixture([.002,.002,.004,.002])
        self.run_arrival(c,clock,sleep)
        self.assertGreaterEqual(clock[0],.21)

    def test_orientation_must_also_settle_and_cancel_is_immediate(self):
        c,clock,seen,sleep=self.fixture([.002],[.03,.01])
        self.run_arrival(c,clock,sleep)
        self.assertGreaterEqual(clock[0],.17)
        c,clock,seen,sleep=self.fixture([.002])
        c.tick.side_effect=InterruptedError('Task canceled')
        with self.assertRaises(InterruptedError):self.run_arrival(c,clock,sleep)
        c.fk.assert_not_called()

    def test_repeated_feedback_stamp_cannot_complete_arrival(self):
        c,clock,seen,sleep=self.fixture([.002])
        c.current=lambda:Sample((0.,)*6,1.,clock[0])
        with self.assertRaises(TimeoutError):self.run_arrival(c,clock,sleep)
        self.assertEqual(len(seen),1)

    def test_task_waits_for_pose_arrival_and_stops_on_pose_timeout(self):
        import tempfile
        from scripts.task.multi_point.task import MultiPointTask
        from scripts.task.common.run_logger import RunLog
        from scripts.task.common.schema import validate_config
        from tests.test_task_ros_messages import TaskMessageTests
        for failed in (False,True):
            with self.subTest(failed=failed),tempfile.TemporaryDirectory() as temp:
                client=TaskMessageTests().client()
                config=validate_config({'points':[self.point()]})
                plan=client.plan
                client.plan=lambda point,start,**kwargs:plan(dict(joints_rad=[0.]*6),start,**kwargs)
                client.stable_pose=Mock(return_value=(client.current(),client.fk()))
                if failed:client.stable_pose.side_effect=TimeoutError('pose did not settle')
                task=MultiPointTask(client,config,{},RunLog(temp),True)
                if failed:
                    with self.assertRaises(TimeoutError):task.run()
                    self.assertEqual(client.stops,1)
                else:
                    result=task.run()
                    self.assertTrue(result['success'])
                    self.assertEqual(result['segments'][0]['actual_pose'],client.fk())
                    self.assertEqual(client.stops,0)
                client.stable_pose.assert_called_once()


if __name__=='__main__':unittest.main()
