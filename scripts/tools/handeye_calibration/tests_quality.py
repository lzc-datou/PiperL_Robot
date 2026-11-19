"""Regression checks for degenerate sampling, source-time matching and stillness."""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from quality import check_observability, interpolate_transform, stationary_window


def transform(xyz=(0, 0, 0), angles=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    result[:3, 3] = xyz
    return result


class QualityChecks(unittest.TestCase):
    def test_degenerate_pose_sets_are_rejected(self):
        for poses in ([transform()] * 20,
                      [transform((i * .01, 0, 0)) for i in range(20)],
                      [transform(angles=(0, 0, i * 3)) for i in range(20)],
                      [transform(angles=(i * .01, i * .01, 0)) for i in range(20)]):
            with self.subTest(poses=len(poses)), self.assertRaisesRegex(ValueError, '旋转不足或退化'):
                check_observability(poses)

    def test_multiaxis_rotations_are_observable(self):
        poses = [transform(angles=(a, b, 0)) for a in (-25, 0, 25) for b in (-20, 0, 20)]
        report = check_observability(poses)
        self.assertGreater(report['pairwise_rotation_max_deg'], 50)
        self.assertGreater(report['second_axis_ratio'], .4)

    def test_interpolation_uses_bracketed_source_times(self):
        history = [(100., transform(angles=(0, 0, 170))),
                   (100.04, transform((.04, 0, 0), (0, 0, -170)))]
        result, metadata = interpolate_transform(history, 100.02)
        np.testing.assert_allclose(result[:3, 3], [.02, 0, 0], atol=1e-10)
        self.assertAlmostEqual(np.degrees(Rotation.from_matrix(result[:3, :3]).magnitude()), 180)
        self.assertAlmostEqual(metadata['interpolation_fraction'], .5)
        for timestamp in (99.99, 100.05):
            with self.assertRaisesRegex(ValueError, '禁止外推'):
                interpolate_transform(history, timestamp)
        with self.assertRaisesRegex(ValueError, '反馈间隔'):
            interpolate_transform(history, 100.02, max_gap_s=.01)

    def test_stillness_needs_source_time_coverage(self):
        repeated = [(100., transform())] * 600
        with self.assertRaisesRegex(ValueError, '完整'):
            stationary_window(repeated, 100.)
        static = [(100 + i * .02, transform((np.sin(i) * .0001, 0, 0),
                                           (0, 0, np.cos(i) * .01))) for i in range(61)]
        report = stationary_window(static, 101.1)
        self.assertGreater(report['unique_source_samples'], 40)
        self.assertLess(report['translation_from_start_mm'], .3)
        missing = [entry for entry in static if not 100.4 < entry[0] < 100.8]
        with self.assertRaisesRegex(ValueError, '反馈缺失'):
            stationary_window(missing, 101.1)

    def test_moving_and_conflicting_feedback_is_rejected(self):
        for axis in ('translation', 'rotation'):
            moving = [(100 + i * .02, transform(
                (i * .0001 if axis == 'translation' else 0, 0, 0),
                (0, 0, i * .01 if axis == 'rotation' else 0))) for i in range(61)]
            with self.subTest(axis=axis), self.assertRaisesRegex(ValueError, '尚未停稳'):
                stationary_window(moving, 101.1)
        with self.assertRaisesRegex(ValueError, '同一反馈源时间戳'):
            interpolate_transform([(100., transform()), (100., transform((.1, 0, 0)))], 100.)


def run():
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(QualityChecks))
    if not result.wasSuccessful():
        raise AssertionError('采样质量回归测试失败。')


if __name__ == '__main__':
    run()
