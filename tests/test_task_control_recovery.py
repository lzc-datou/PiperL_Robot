"""Gate recovery ordering; fixtures never create ROS nodes or a CAN driver."""
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
try:
    from scripts.task.common.moveit_client import MoveItClient
    ROS=True
except ImportError:
    ROS=False

@unittest.skipUnless(ROS,'Use ROS runtime wrapper')
class RecoveryTests(unittest.TestCase):
    def client(self):
        q=(.1,.2,-.3,.4,.5,.6)
        events=[]
        c=SimpleNamespace(task_gate_closed=True,motion_submitted=False,
            settings={'feedback_timeout':.5,'start_tolerance_rad':.01},gate=object(),
            node=Mock(get_node_names_and_namespaces=Mock(return_value=[])),
            begin_round=Mock(),stable=Mock(return_value=SimpleNamespace(q=q)),
            current=Mock(return_value=SimpleNamespace(q=q)),tick=Mock(),
            command_feedback=Mock(get=Mock(return_value=SimpleNamespace(q=q))))
        def call(service,request,timeout):
            events.append(('gate',request.data));return SimpleNamespace(success=True,message='')
        def execute(trajectory,start,duration):
            self.assertEqual(events,[('gate',False)])
            self.assertEqual(list(trajectory.joint_trajectory.points[0].positions),list(q))
            self.assertEqual(list(trajectory.joint_trajectory.points[0].velocities),[0.]*6)
            events.append(('align',tuple(start)))
        c.call=Mock(side_effect=call);c.execute=Mock(side_effect=execute)
        c.stop=Mock(side_effect=lambda:events.append(('stop',False)))
        return c,events

    def test_gate_opens_only_after_isolated_hold_alignment(self):
        c,events=self.client();MoveItClient.restore_task_control(c)
        self.assertEqual([x[0] for x in events],['gate','align','gate'])
        self.assertEqual(events[-1],('gate',True));self.assertFalse(c.task_gate_closed)
        c.stop.assert_not_called()

    def test_unresolved_goal_never_reopens_or_aligns(self):
        c,events=self.client();c.begin_round.side_effect=RuntimeError('pending goal')
        with self.assertRaises(RuntimeError):MoveItClient.restore_task_control(c)
        self.assertEqual(events,[])

    def test_mismatched_or_stale_hold_never_reopens(self):
        for stale in (False,True):
            c,events=self.client()
            if stale:c.command_feedback.get.side_effect=RuntimeError('stale')
            else:c.command_feedback.get.return_value.q=(9.,)*6
            with self.assertRaises(RuntimeError):MoveItClient.restore_task_control(c)
            self.assertNotIn(('gate',True),events);c.stop.assert_called_once()
            self.assertTrue(c.task_gate_closed)

    def test_operator_gate_is_not_reopened_by_task(self):
        c,events=self.client();c.task_gate_closed=False
        MoveItClient.restore_task_control(c);self.assertEqual(events,[])

    def test_failed_close_or_external_auto_gate_prevents_alignment(self):
        c,events=self.client();c.call.return_value=None;c.call.side_effect=lambda *a:SimpleNamespace(success=False)
        with self.assertRaises(RuntimeError):MoveItClient.restore_task_control(c)
        c.execute.assert_not_called()
        c,events=self.client();c.node.get_node_names_and_namespaces.return_value=[('control_gate','/')]
        with self.assertRaises(RuntimeError):MoveItClient.restore_task_control(c)
        c.call.assert_not_called()

    def test_uncertain_open_is_closed_again_and_remains_locked(self):
        c,events=self.client();call=c.call.side_effect
        def timeout(service,request,wait):
            if request.data:raise TimeoutError('open acknowledgement lost')
            return call(service,request,wait)
        c.call.side_effect=timeout
        with self.assertRaises(TimeoutError):MoveItClient.restore_task_control(c)
        c.stop.assert_called_once();self.assertTrue(c.task_gate_closed)

    def test_stop_during_gate_open_closes_again(self):
        c,events=self.client();call=c.call.side_effect
        def stop_on_open(service,request,wait):
            result=call(service,request,wait)
            if request.data:c.tick.side_effect=InterruptedError('stop')
            return result
        c.call.side_effect=stop_on_open
        with self.assertRaises(InterruptedError):MoveItClient.restore_task_control(c)
        self.assertEqual(events[-1],('stop',False))
        c.stop.assert_called_once();self.assertTrue(c.task_gate_closed)

if __name__=='__main__':unittest.main()
