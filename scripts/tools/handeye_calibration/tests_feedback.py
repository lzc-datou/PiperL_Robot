"""Offline regression checks for namespaced TCP feedback selection and discovery."""
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import feedback


def publisher(namespace='/', name='agx_arm_ctrl_single_node', topic_type=feedback.POSE_TYPE):
    return SimpleNamespace(node_namespace=namespace, node_name=name, topic_type=topic_type)


class DiscoveryFixture:
    """Advance a bounded fake clock so an accidental infinite wait fails quickly."""
    def __init__(self, publishers_at, shutdown_after=float('inf'), interrupt_after=float('inf')):
        self.elapsed = 0.
        self.messages, self.graph_queries = [], []
        self.publishers_at = publishers_at
        self.shutdown_after, self.interrupt_after = shutdown_after, interrupt_after
        def log(level):
            return lambda message: self.messages.append((level, message, self.elapsed))
        logger = SimpleNamespace(info=log('info'), warning=log('warning'), warn=log('warning'))
        self.node = SimpleNamespace(get_publishers_info_by_topic=self.publishers,
            get_logger=lambda: logger, context=SimpleNamespace(get_domain_id=lambda: 43))
        self.rclpy = SimpleNamespace(spin_once=self.spin_once,
            ok=lambda *args, **kwargs: self.elapsed < self.shutdown_after)

    def publishers(self, topic):
        self.graph_queries.append((topic, self.elapsed))
        return self.publishers_at(self.elapsed).get(topic, [])

    def spin_once(self, node, timeout_sec):
        self.elapsed += timeout_sec
        if self.elapsed >= self.interrupt_after:
            raise KeyboardInterrupt
        if self.elapsed > 30:
            raise AssertionError('Discovery did not finish within the bounded fake test window.')

    def resolve(self, **kwargs):
        with patch.dict(sys.modules, {'rclpy': self.rclpy}), \
                patch.object(feedback.time, 'monotonic', side_effect=lambda: self.elapsed):
            return feedback.resolve_robot_pose_topic(self.node, **kwargs)


class FeedbackChecks(unittest.TestCase):
    def test_existing_single_point_and_gamepad_sources(self):
        for topic, namespace in zip(feedback.ROBOT_POSE_TOPICS, ('/', '/piper_servo')):
            with self.subTest(topic=topic):
                self.assertEqual(feedback.select_robot_pose_topic('auto', {topic: [publisher(namespace)]}), topic)

    def test_two_topics_or_two_publishers_are_ambiguous(self):
        with self.assertRaisesRegex(ValueError, '多个机器人 TCP') as caught:
            feedback.select_robot_pose_topic('auto', {topic: [publisher()] for topic in feedback.ROBOT_POSE_TOPICS})
        for topic in feedback.ROBOT_POSE_TOPICS:
            self.assertIn(topic, str(caught.exception))
        with self.assertRaisesRegex(ValueError, '多个发布者'):
            feedback.select_robot_pose_topic('auto', {feedback.ROBOT_POSE_TOPICS[0]: [publisher(), publisher(name='other')]})

    def test_wrong_type_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '类型不匹配'):
            feedback.select_robot_pose_topic('auto', {
                feedback.ROBOT_POSE_TOPICS[0]: [publisher()],
                feedback.ROBOT_POSE_TOPICS[1]: [publisher(topic_type='sensor_msgs/msg/JointState')]})

    def test_no_source_does_not_select_tf_or_commanded_feedback(self):
        with self.assertRaisesRegex(ValueError, '未发现') as caught:
            feedback.select_robot_pose_topic('auto', {
                '/piper_servo/commanded_joint_states': [publisher(topic_type='sensor_msgs/msg/JointState')],
                '/piper_servo/tf': [publisher(topic_type='tf2_msgs/msg/TFMessage')]})
        for topic in feedback.ROBOT_POSE_TOPICS:
            self.assertIn(topic, str(caught.exception))
        self.assertNotIn('Domain', str(caught.exception), 'Missing publishers alone do not establish a Domain mismatch.')

    def test_explicit_topic_is_respected_and_must_be_absolute(self):
        self.assertEqual(feedback.select_robot_pose_topic('/my_arm/actual_tcp', {}), '/my_arm/actual_tcp')
        for invalid in ('feedback/tcp_pose', '/bad//pose', '/pose/', '/2arm/pose', '/bad name', '', None):
            with self.subTest(configured=invalid), self.assertRaises(ValueError):
                feedback.select_robot_pose_topic(invalid, {})

    def test_discovery_waits_full_window_and_reports_publisher(self):
        topic = feedback.ROBOT_POSE_TOPICS[1]
        fixture = DiscoveryFixture(lambda elapsed: {topic: [publisher('/piper_servo', 'renamed_official_driver')]})
        self.assertEqual(fixture.resolve(), topic)
        self.assertAlmostEqual(fixture.elapsed, 2.)
        self.assertEqual(len(fixture.graph_queries), 2)
        self.assertTrue(all(when >= 2. for _, when in fixture.graph_queries))
        messages = '\n'.join(message for _, message, _ in fixture.messages)
        self.assertIn(topic, messages)
        self.assertIn('/piper_servo/renamed_official_driver', messages)

    def test_missing_source_waits_ten_seconds_then_accepts_late_publisher(self):
        topic = feedback.ROBOT_POSE_TOPICS[1]
        fixture = DiscoveryFixture(lambda elapsed: {topic: [publisher('/piper_servo')]} if elapsed >= 10. else {})
        self.assertEqual(fixture.resolve(), topic)
        self.assertGreaterEqual(fixture.elapsed, 10.)
        waiting = [(message, when) for _, message, when in fixture.messages if '未发现' in message]
        self.assertGreaterEqual(len(waiting), 1)
        self.assertLessEqual(len(waiting), 3, 'Missing-source status must be throttled, not printed each poll.')
        self.assertTrue(any('当前 ROS Domain 43' in message for _, message, _ in fixture.messages))
        self.assertTrue(all('ROS Domain' not in message for message, _ in waiting),
                        'Domain is startup context, not a concluded cause of missing feedback.')
        for (previous_message, previous), (message, current) in zip(waiting, waiting[1:]):
            self.assertGreaterEqual(current - previous, 5. - 1e-9)

    def test_conflicting_sources_wait_until_the_graph_recovers(self):
        single, gamepad = feedback.ROBOT_POSE_TOPICS
        for conflict in ({single: [publisher()], gamepad: [publisher('/piper_servo')]},
                         {gamepad: [publisher('/piper_servo'), publisher('/piper_servo', 'other')]},
                         {gamepad: [publisher('/piper_servo', topic_type='sensor_msgs/msg/JointState')]}):
            with self.subTest(conflict=conflict):
                fixture = DiscoveryFixture(lambda elapsed: conflict if elapsed < 6. else {gamepad: [publisher('/piper_servo')]})
                self.assertEqual(fixture.resolve(), gamepad)
                self.assertGreaterEqual(fixture.elapsed, 6.)
                self.assertTrue(any(level == 'warning' for level, _, _ in fixture.messages))

    def test_ros_shutdown_ends_wait_cleanly(self):
        fixture = DiscoveryFixture(lambda elapsed: {}, shutdown_after=3.)
        with self.assertRaises(SystemExit) as caught:
            fixture.resolve()
        self.assertEqual(caught.exception.code, 0)
        self.assertGreaterEqual(fixture.elapsed, 3.)
        self.assertLess(fixture.elapsed, 3.1)

    def test_keyboard_interrupt_remains_cancellable(self):
        fixture = DiscoveryFixture(lambda elapsed: {}, interrupt_after=3.)
        with self.assertRaises(KeyboardInterrupt):
            fixture.resolve()
        self.assertLess(fixture.elapsed, 3.1)

    def test_explicit_resolver_skips_graph_discovery(self):
        messages = []
        node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=messages.append))
        self.assertEqual(feedback.resolve_robot_pose_topic(node, '/explicit/tcp'), '/explicit/tcp')
        self.assertIn('/explicit/tcp', messages[0])


def run():
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(FeedbackChecks))
    if not result.wasSuccessful():
        raise AssertionError('机器人反馈话题选择回归测试失败。')


if __name__ == '__main__':
    run()
