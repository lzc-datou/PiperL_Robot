import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from scripts.tools.monitoring.point_panel import PointPanel
from scripts.tools.monitoring.read_joint_angles import JointFeedback

class PointPanelTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        with patch('scripts.tools.monitoring.point_panel.ReadOnlyFK',side_effect=ImportError('no ROS')),patch('sys.stdin.isatty',return_value=False):
            self.panel=PointPanel(Path(self.temp.name),snapshot_dir=Path(self.temp.name)/'points')
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

if __name__=='__main__':unittest.main()
