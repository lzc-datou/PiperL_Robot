"""Boundary diagnostics without a ROS node or robot connection."""
import unittest
from types import SimpleNamespace
try:
    from scripts.task.common.moveit_client import MoveItClient
    from scripts.task.common.schema import JOINTS
    from scripts.task.common.joint_bounds import JointBoundsError
    ROS=True
except ImportError:
    ROS=False


@unittest.skipUnless(ROS,'Use ROS runtime wrapper')
class JointBoundsTests(unittest.TestCase):
    def test_reports_all_violations_without_modifying_input(self):
        client=SimpleNamespace(limits={name:(0.,1.) for name in JOINTS})
        q=[0.,-.045361,1.2,0.,0.,0.]
        with self.assertRaises(JointBoundsError) as caught:
            MoveItClient.check_joints(client,q)
        self.assertEqual([v['joint'] for v in caught.exception.details['violations']],['joint2','joint3'])
        self.assertIn('-2.599°',str(caught.exception))
        self.assertEqual(q,[0.,-.045361,1.2,0.,0.,0.])

    def test_exact_boundary_allowed_but_no_implicit_tolerance(self):
        client=SimpleNamespace(limits={name:(0.,1.) for name in JOINTS})
        MoveItClient.check_joints(client,[0.,1.,0.,1.,0.,1.])
        with self.assertRaises(JointBoundsError):
            MoveItClient.check_joints(client,[0.,-1e-9,0.,0.,0.,0.])
