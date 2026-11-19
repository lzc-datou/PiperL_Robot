"""Synthetic regressions for a shared hand-eye transform after moving the board."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from combined import solve_grouped
from verify import dataset, evaluate, training_spread


def transform(angles, translation):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
    result[:3, 3] = translation
    return result


def synthetic_groups(counts=(30, 28), seed=58):
    rng = np.random.default_rng(seed)
    expected = transform([.2, -.3, 1.1], [-.09, -.02, .04])
    boards = [transform([.1, .2, .3], [.4, .1, .2]),
              transform([-.6, 1., -1.], [-1.2, 1.8, .6])]
    groups = []
    for group_index, count in enumerate(counts):
        samples = []
        for sample_index in range(count):
            robot = transform(rng.uniform(-.8, .8, 3), rng.uniform(-.3, .3, 3))
            marker = np.linalg.inv(expected) @ np.linalg.inv(robot) @ boards[group_index]
            samples.append({'base_T_end': robot.tolist(),
                            'camera_T_marker': marker.tolist(),
                            'source': {'sample': sample_index, 'group': group_index}})
        groups.append(samples)
    return expected, boards[:len(counts)], groups


def opencv_park(samples):
    robots = [np.asarray(sample['base_T_end']) for sample in samples]
    markers = [np.asarray(sample['camera_T_marker']) for sample in samples]
    rotation, translation = cv2.calibrateHandEye(
        [robot[:3, :3] for robot in robots], [robot[:3, 3] for robot in robots],
        [marker[:3, :3] for marker in markers], [marker[:3, 3] for marker in markers],
        method=cv2.CALIB_HAND_EYE_PARK)
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, translation.reshape(3)
    return result


def grouped_samples(groups):
    return [{**sample, 'combined_provenance': {'fixed_board_group': group_number}}
            for group_number, group in enumerate(groups, 1) for sample in group]


def saved_dataset(folder, samples, transform=None, group_count=1):
    folder.mkdir()
    metadata = {'mode': 'eye_in_hand', 'fixed_board_groups': group_count}
    (folder / 'session.json').write_text(json.dumps(metadata))
    (folder / 'samples.json').write_text(json.dumps(samples))
    if transform is not None:
        (folder / 'T_end_camera.json').write_text(json.dumps(transform.tolist()))
    return dataset(folder, require_result=transform is not None)


class CombinedChecks(unittest.TestCase):
    def test_moved_board_groups_recover_one_shared_transform(self):
        expected, boards, groups = synthetic_groups()
        self.assertGreater(np.linalg.norm(boards[0][:3, 3] - boards[1][:3, 3]), 2.)
        actual, diagnostics = solve_grouped(groups)
        np.testing.assert_allclose(actual, expected, atol=1e-10)
        self.assertIsInstance(diagnostics, dict)
        for group, board in zip(groups, boards):
            for sample in group:
                recovered = np.asarray(sample['base_T_end']) @ actual @ np.asarray(sample['camera_T_marker'])
                np.testing.assert_allclose(recovered, board, atol=1e-10)

    def test_single_group_matches_opencv_park_and_known_truth(self):
        expected, _, groups = synthetic_groups(counts=(24,))
        actual, _ = solve_grouped(groups)
        np.testing.assert_allclose(actual, expected, atol=1e-10)
        np.testing.assert_allclose(actual, opencv_park(groups[0]), atol=1e-10)

    def test_reordering_groups_and_samples_preserves_result(self):
        _, _, groups = synthetic_groups()
        original, _ = solve_grouped(groups)
        rng = np.random.default_rng(1988)
        reordered = [[group[index] for index in rng.permutation(len(group))]
                     for group in reversed(groups)]
        actual, _ = solve_grouped(reordered)
        np.testing.assert_allclose(actual, original, atol=1e-10)

    def test_translation_and_single_axis_rotations_are_rejected(self):
        expected = transform([.2, -.3, 1.1], [-.09, -.02, .04])
        board = transform([.1, .2, .3], [.4, .1, .2])
        for single_axis in (False, True):
            group = []
            for index in range(20):
                robot = transform([0., 0., index * .04 if single_axis else 0.],
                                  [index * .01, .01 * np.sin(index), 0.])
                marker = np.linalg.inv(expected) @ np.linalg.inv(robot) @ board
                group.append({'base_T_end': robot.tolist(), 'camera_T_marker': marker.tolist()})
            with self.subTest(single_axis=single_axis), self.assertRaises(ValueError):
                solve_grouped([group])

    def test_inputs_including_source_metadata_are_not_modified(self):
        _, _, groups = synthetic_groups()
        original = copy.deepcopy(groups)
        solve_grouped(groups)
        self.assertEqual(groups, original)

    def test_distinct_group_rotation_axes_jointly_constrain_shared_transform(self):
        expected, boards, _ = synthetic_groups()
        groups = []
        for group_index, axis in enumerate((0, 2)):
            group = []
            for index in range(20):
                angles = np.zeros(3)
                angles[axis] = -.4 + index * .04
                robot = transform(angles, [index * .01, .01 * np.sin(index), 0.])
                marker = np.linalg.inv(expected) @ np.linalg.inv(robot) @ boards[group_index]
                group.append({'base_T_end': robot.tolist(), 'camera_T_marker': marker.tolist()})
            groups.append(group)
        actual, _ = solve_grouped(groups)
        np.testing.assert_allclose(actual, expected, atol=1e-10)

    def test_noisy_single_group_rotation_matches_opencv_park(self):
        _, _, groups = synthetic_groups(counts=(24,))
        rng = np.random.default_rng(42)
        for sample in groups[0]:
            for key in ('base_T_end', 'camera_T_marker'):
                matrix = np.asarray(sample[key])
                perturbation = Rotation.from_rotvec(rng.normal(0., .001, 3)).as_matrix()
                matrix[:3, :3] = matrix[:3, :3] @ perturbation
                matrix[:3, 3] += rng.normal(0., .0002, 3)
                sample[key] = matrix.tolist()
        actual, _ = solve_grouped(groups)
        # Translation is intentionally estimated from absolute samples after
        # rotation; its noise objective differs from OpenCV's pairwise equation.
        np.testing.assert_allclose(actual[:3, :3], opencv_park(groups[0])[:3, :3], atol=1e-8)

    def test_training_spread_centers_moved_board_groups_separately(self):
        expected, _, groups = synthetic_groups()
        data = {'samples': grouped_samples(groups), 'metadata': {'fixed_board_groups': 2}}
        result = training_spread(data, expected)
        self.assertEqual(result['samples'], 58)
        self.assertEqual(len(result['fixed_board_groups']), 2)
        self.assertLess(result['position_rms_mm'], 1e-8)
        self.assertLess(result['position_max_mm'], 1e-8)
        self.assertLess(result['rotation_rms_deg'], 1e-8)

    def test_training_spread_aggregates_squared_residuals_per_observation(self):
        expected, _, groups = synthetic_groups(counts=(2, 4))
        offsets = ([0., .002], [0., 0., .006, .006])
        for group, deviations in zip(groups, offsets):
            for sample, deviation in zip(group, deviations):
                sample['base_T_end'][0][3] += deviation
        data = {'samples': grouped_samples(groups), 'metadata': {'fixed_board_groups': 2}}
        result = training_spread(data, expected)
        self.assertAlmostEqual(result['position_rms_mm'], np.sqrt((2 * 1. ** 2 + 4 * 3. ** 2) / 6))
        self.assertAlmostEqual(result['position_max_mm'], 3.)

    def test_combined_calibration_can_validate_a_single_board_dataset(self):
        expected, _, groups = synthetic_groups()
        _, _, validation_groups = synthetic_groups(counts=(10,), seed=7)
        with tempfile.TemporaryDirectory(prefix='piperl-combined-verify-') as directory:
            calibration = saved_dataset(Path(directory) / 'calibration', grouped_samples(groups),
                                        expected, group_count=2)
            validation = saved_dataset(Path(directory) / 'validation', validation_groups[0])
            result = evaluate(calibration, validation)
            self.assertLess(result['training']['position_rms_mm'], 1e-8)
            self.assertLess(result['validation']['position_rms_mm'], 1e-8)
            self.assertNotIn('validation_to_reference', result)
            self.assertEqual(result['overlapping_validation_sample_numbers'], [])
            with self.assertRaisesRegex(ValueError, '多个棋盘格位置'):
                evaluate(calibration, validation, reference_data=calibration)
            with self.assertRaisesRegex(ValueError, '多个棋盘格位置'):
                evaluate(calibration, calibration)


def run():
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(CombinedChecks)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():
        raise AssertionError('多板位置联合标定回归测试失败。')


if __name__ == '__main__':
    run()
