"""Preview may reacquire fresh feedback; execution must never reuse stale samples."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from scripts.task.common.robot_feedback import Feedback,Sample,FeedbackUnavailableError


class FeedbackDiagnosticsTests(unittest.TestCase):
    def test_diagnostic_distinguishes_missing_source_receive_and_clock(self):
        feedback=Feedback()
        for sample,now,wall,reason in (
                (None,10.,100.,'missing'),
                (Sample((0.,)*6,100.,9.),10.,100.1,'receive_stale'),
                (Sample((0.,)*6,99.,10.),10.,100.,'source_stale'),
                (Sample((0.,)*6,101.,10.),10.,100.,'future_timestamp')):
            feedback.sample=sample
            with self.assertRaises(FeedbackUnavailableError) as caught:
                feedback.get(.5,now=now,wall=wall)
            self.assertEqual(caught.exception.details['reason'],reason)
            self.assertEqual(caught.exception.details['timeout_seconds'],.5)


try:
    from scripts.task.common.moveit_client import MoveItClient
    from scripts.task.common.schema import DEFAULTS
    ROS=True
except ImportError:
    ROS=False


@unittest.skipUnless(ROS,'Use ROS runtime wrapper')
class PreviewFeedbackWaitTests(unittest.TestCase):
    def fixture(self,available):
        clock=[0.]
        error=FeedbackUnavailableError(dict(reason='receive_stale',kind='feedback_unavailable',
            receive_age_seconds=.6,source_age_seconds=.6,timeout_seconds=.5))
        def current():
            if not available(clock[0]):raise error
            return Sample((0.,)*6,100.+clock[0],clock[0])
        client=SimpleNamespace(settings=dict(DEFAULTS,settle_seconds=.2),tick=Mock(),current=Mock(side_effect=current))
        return client,clock,error

    def run_stable(self,client,clock,**kwargs):
        def sleep(value):clock[0]+=value
        with patch('scripts.task.common.moveit_client.time.monotonic',side_effect=lambda:clock[0]), \
             patch('scripts.task.common.moveit_client.time.sleep',side_effect=sleep):
            return MoveItClient.stable(client,**kwargs)

    def test_preview_resets_settle_window_after_a_gap(self):
        client,clock,error=self.fixture(lambda now:now<.18 or now>=.30)
        result=self.run_stable(client,clock,timeout=1.,health=False)
        self.assertGreaterEqual(result.received,.5)
        self.assertLess(result.received,1.)

    def test_persistent_outage_is_bounded_and_keeps_diagnostic(self):
        client,clock,error=self.fixture(lambda now:False)
        with self.assertRaises(FeedbackUnavailableError) as caught:
            self.run_stable(client,clock,timeout=.2,health=False)
        self.assertIs(caught.exception,error)
        self.assertGreaterEqual(clock[0],.2)
        self.assertLess(clock[0],.23)

    def test_execution_and_target_arrival_do_not_wait_through_stale_feedback(self):
        for options in (dict(health=True),dict(health=False,target=[0.]*6)):
            client,clock,error=self.fixture(lambda now:False)
            with self.assertRaises(FeedbackUnavailableError):self.run_stable(client,clock,timeout=1.,**options)
            self.assertEqual(clock[0],0.)
            self.assertEqual(client.current.call_count,1)

    def test_invalid_feedback_and_future_clock_fail_without_retry(self):
        for reason in ('invalid','future_timestamp'):
            client,clock,error=self.fixture(lambda now:False)
            if reason=='invalid':client.current.side_effect=RuntimeError('bad joint names')
            else:error.details['reason']='future_timestamp'
            with self.assertRaises(RuntimeError):self.run_stable(client,clock,timeout=1.,health=False)
            self.assertEqual(clock[0],0.)
