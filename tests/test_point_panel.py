import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import yaml
from scripts.task.common.schema import load_config
from scripts.tools.monitoring.point_panel import PointPanel
from scripts.tools.monitoring.read_joint_angles import JointFeedback, ROOT

class PointPanelTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        with patch('scripts.tools.monitoring.point_panel.ReadOnlyFK',side_effect=ImportError('no ROS')),patch('sys.stdin.isatty',return_value=False):
            self.panel=PointPanel(Path(self.temp.name),snapshot_dir=Path(self.temp.name)/'points',fk_source='moveit')
        self.addCleanup(self.panel.close)
        self.panel.keys=lambda:''
        self.feedback=JointFeedback()
        self.feedback.degrees=[0.,10.,-20.,0.,0.,0.]
    def render(self,now,key=''):
        self.panel.keys=lambda:key
        self.feedback.received_at=[now-.001]*6
        return self.panel.render(self.feedback,now,.5)
    def test_freeze_keeps_point_while_table_source_changes_then_save(self):
        now=time.monotonic()
        for offset in (0,.1,.2,.31): self.render(now+offset)
        text=self.render(now+.32,'p')
        self.assertIn('FROZEN',text)
        self.assertIsNotNone(self.panel.frozen)
        original=self.panel.frozen['joint']['points'][0]['joints_rad'][:]
        self.feedback.degrees[0]=25
        self.render(now+.4)
        self.assertEqual(self.panel.frozen['joint']['points'][0]['joints_rad'],original)
        self.render(now+.5,'s')
        self.assertEqual(len(list((Path(self.temp.name)/'points').glob('*/joint.yaml'))),1)
        self.assertEqual(len(list((Path(self.temp.name)/'points').glob('*/pose.yaml'))),0)
        self.render(now+.6,'r')
        self.assertIsNone(self.panel.frozen)
    def test_moving_and_stale_frames_cannot_be_captured(self):
        now=time.monotonic()
        self.render(now,'s')
        self.assertIsNone(self.panel.frozen)
        self.feedback.received_at=[now-2]*6
        self.panel.keys=lambda:'s'
        self.panel.render(self.feedback,now,.5)
        self.assertIsNone(self.panel.frozen)
        self.assertFalse((Path(self.temp.name)/'points').exists())

    def test_feedback_warning_clears_after_complete_fresh_sample(self):
        now=time.monotonic()
        for mode,warning in [('missing','Position feedback incomplete'),
                             ('stale','Position feedback stale'),
                             ('skew','Joint-pair feedback skew too large')]:
            with self.subTest(mode=mode):
                self.feedback.received_at=[now-.001]*6
                if mode=='missing': self.feedback.received_at[0]=None
                elif mode=='stale': self.feedback.received_at[0]=now-2
                else: self.feedback.received_at[0]=now-.1
                text=self.panel.render(self.feedback,now,.5)
                self.assertIn(warning,text)
                recovered=self.render(now+.01)
                self.assertNotIn(warning,recovered)
                self.assertIn('joints_rad:',recovered)
    def test_transient_feedback_warning_does_not_erase_saved_path(self):
        now=time.monotonic()
        for offset in (0,.1,.2,.31): self.render(now+offset)
        self.render(now+.32,'s')
        saved=self.panel.note
        self.assertTrue(saved.startswith('Saved: '))
        self.panel.keys=lambda:''
        self.feedback.received_at=[now-2]*6
        bad=self.panel.render(self.feedback,now+.4,.5)
        self.assertIn('Position feedback stale',bad)
        self.assertNotIn(saved,bad)
        good=self.render(now+.5)
        self.assertNotIn('Position feedback stale',good)
        self.assertIn(saved,good)

    def test_local_fk_freezes_one_cartesian_pose_and_saves_both_orientations(self):
        with patch('sys.stdin.isatty', return_value=False):
            local_panel = PointPanel(ROOT, snapshot_dir=Path(self.temp.name)/'points')
        self.addCleanup(local_panel.close)
        self.panel = local_panel
        now = time.monotonic()
        self.feedback.degrees = [0.]*6
        for offset in (0., .1, .2, .31):
            self.render(now+offset)
        frozen = self.render(now+.33, 'p')
        self.assertIn('FROZEN', frozen)
        self.assertIn('position_m: [0.073061, 0.000000, 0.227595]', frozen)
        self.assertRegex(frozen, r'orientation_xyzw: \[[^\n]+\],\n +rpy: \[')
        self.assertEqual(frozen.count('type: pose'), 1)
        self.assertNotIn('XYZ (mm)', frozen)
        self.assertNotIn('xyzrpy:', frozen)
        self.feedback.degrees[0] = 25.
        saved = self.render(now+.43, 's')
        self.assertIn('position_m: [0.073061, 0.000000, 0.227595]', saved)
        directory, = local_panel.directory.iterdir()
        self.assertEqual({path.name for path in directory.iterdir()}, {'joint.yaml', 'pose.yaml'})
        joint = yaml.safe_load((directory/'joint.yaml').read_text())
        pose = yaml.safe_load((directory/'pose.yaml').read_text())
        self.assertEqual(joint['capture'], pose['capture'])
        self.assertEqual(joint['points'][0]['joints_rad'], [0.]*6)
        self.assertEqual(joint['capture']['kinematics_sha256'], local_panel.local_fk.kinematics_sha256)
        self.assertAlmostEqual(pose['points'][0]['position_m'][0], .07306107)
        self.assertAlmostEqual(pose['points'][0]['rpy'][1], 85., places=4)
        goal = load_config(directory/'pose.yaml')['points'][0]
        self.assertEqual(goal['position_m'], pose['points'][0]['position_m'])
        local_panel.keys = lambda: 'r'
        self.feedback.received_at = [now-2] * 6
        expired = local_panel.render(self.feedback, now+.53, .5)
        self.assertIn('Position feedback stale', expired)
        self.assertNotIn('rpy:', expired)
        self.assertNotIn('orientation_xyzw:', expired)
        resume = self.render(now+.63)
        self.assertIn('LIVE', resume)
        self.assertIn('rpy:', resume)
        self.assertIn('orientation_xyzw:', resume)
        self.assertNotIn('Position feedback stale', resume)
        self.assertNotIn('position_m: [0.073061, 0.000000, 0.227595]', resume)

if __name__=='__main__':unittest.main()
