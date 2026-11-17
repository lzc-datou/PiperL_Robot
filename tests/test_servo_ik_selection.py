"""Pure IK seed/ranking math; no ROS imports, service requests or motion."""
import math
import unittest

import numpy as np

from scripts.task.servo_tracking.ik_selection import (
    make_ik_seeds, candidate_metrics, candidate_score, select_ik_candidate)


class ServoIKSelectionTests(unittest.TestCase):
    def setUp(self):
        self.lower = np.array([-2.6179938, 0., -2.9670597, -2.2165681, -1.5620696, -2.9670597])
        self.upper = np.array([2.6179938, 3.1415926, 0., 2.2165681, 1.5620696, 2.9670597])
        self.reference = np.array([.1, 1.4, -1.2, .3, 1., 0.])

    def seeds(self, **kwargs):
        return make_ik_seeds(self.reference, self.lower, self.upper, **kwargs)

    def test_seed_order_previous_ready_middle_and_distinct_wrist_guesses(self):
        previous, ready = self.reference.copy(), self.reference.copy()
        previous[0] += .1
        ready[4] -= .1
        seeds = self.seeds(previous=previous, ready=ready)
        self.assertEqual(len(seeds), 6)
        for actual, expected in zip(seeds[:4],
                [self.reference, previous, ready, (self.lower+self.upper)*.5]):
            np.testing.assert_array_equal(actual, expected)
        self.assertNotEqual(seeds[4][3], self.reference[3])
        self.assertNotEqual(seeds[5][3], seeds[4][3])
        self.assertAlmostEqual(seeds[4][4], -self.reference[4])
        for seed in seeds:
            self.assertTrue(np.all(seed >= self.lower) and np.all(seed <= self.upper))

    def test_all_seeds_clamp_finite_guesses_and_do_not_change_inputs(self):
        reference = self.reference.copy()
        reference[1] = -.0004
        previous, ready = np.full(6, 100.), np.full(6, -100.)
        originals = [value.copy() for value in (reference, previous, ready)]
        seeds = make_ik_seeds(reference, self.lower, self.upper,
                              previous=previous, ready=ready, margin=.015)
        for seed in seeds:
            self.assertTrue(np.all(seed >= self.lower+.015))
            self.assertTrue(np.all(seed <= self.upper-.015))
        for actual, original in zip((reference, previous, ready), originals):
            np.testing.assert_array_equal(actual, original)

    def test_duplicate_and_malformed_optional_guesses_do_not_replace_reference(self):
        seeds = self.seeds(previous=self.reference, ready=[float('nan')]*6)
        np.testing.assert_array_equal(seeds[0], self.reference)
        self.assertEqual(sum(np.allclose(seed, self.reference) for seed in seeds), 1)
        self.assertEqual(len(seeds), 4)
        short = self.seeds(previous=[0.], ready=[None]*6, max_seeds=1)
        self.assertEqual(len(short), 1)

    def test_exact_wrist_flip_is_a_bounded_seed_for_observed_branch(self):
        reference = np.array([.0974, 1.4766, -1.0178, 2.1975, -.4283, 1.8209])
        seeds = make_ik_seeds(reference, self.lower, self.upper)
        expected = reference.copy()
        expected[3] -= math.pi
        expected[4] *= -1.
        expected[5] -= math.pi
        self.assertTrue(any(np.allclose(seed, expected, rtol=0, atol=1e-10) for seed in seeds))
        self.assertEqual(sum(np.allclose(seed, expected, rtol=0, atol=1e-10) for seed in seeds), 1)

    def test_nearby_branch_wins_over_large_wrist_jump_despite_more_margin(self):
        reference = self.reference.copy()
        reference[3], reference[4], reference[5] = 2.19, -.428, 1.82
        nearby = reference.copy()
        nearby[3] -= .002
        flipped = nearby.copy()
        flipped[3] -= math.pi
        flipped[4] *= -1.
        flipped[5] -= math.pi
        self.assertGreater(candidate_metrics(flipped, reference, self.lower, self.upper)['normalized_margin'],
                           candidate_metrics(nearby, reference, self.lower, self.upper)['normalized_margin'])
        np.testing.assert_array_equal(select_ik_candidate([flipped, nearby], reference,
            self.lower, self.upper, margin=.015), nearby)

    def test_equal_travel_prefers_more_normalized_limit_margin(self):
        lower, upper, reference = np.full(6, -2.), np.full(6, 2.), np.full(6, 1.5)
        less_room, more_room = np.full(6, 1.4), np.full(6, 1.4)
        less_room[0] = 1.6
        np.testing.assert_array_equal(select_ik_candidate([less_room, more_room], reference,
            lower, upper), more_room)

    def test_bounded_joint_continuity_never_wraps_across_a_hard_stop(self):
        reference = np.zeros(6)
        lower, upper = np.full(6, -math.pi), np.full(6, math.pi)
        reference[5] = math.pi-.01
        wrapped, nearby = reference.copy(), reference.copy()
        wrapped[5], nearby[5] = -math.pi+.01, math.pi-.15
        metric = candidate_metrics(wrapped, reference, lower, upper)
        self.assertGreater(metric['continuity_max'], .99)
        np.testing.assert_array_equal(select_ik_candidate([wrapped, nearby], reference, lower, upper), nearby)

    def test_candidates_are_excluded_not_clamped_and_result_is_independent_copy(self):
        outside = self.reference.copy()
        outside[1] = -.001
        nan = self.reference.copy()
        nan[2] = float('nan')
        candidates = [outside, nan, [0.], self.reference]
        chosen = select_ik_candidate(candidates, self.reference, self.lower, self.upper, margin=.015)
        np.testing.assert_array_equal(chosen, self.reference)
        chosen[0] += .1
        self.assertNotEqual(chosen[0], self.reference[0])
        self.assertEqual(outside[1], -.001)

    def test_soft_margin_is_enforced_and_no_legal_candidate_fails_closed(self):
        at_boundary = self.reference.copy()
        at_boundary[1] = self.lower[1]
        self.assertTrue(math.isfinite(candidate_score(at_boundary, self.reference, self.lower, self.upper)))
        with self.assertRaises(ValueError):
            candidate_score(at_boundary, self.reference, self.lower, self.upper, margin=.015)
        for candidates in ([], [at_boundary], [[float('nan')]*6]):
            with self.assertRaises(ValueError):
                select_ik_candidate(candidates, self.reference, self.lower, self.upper, margin=.015)

    def test_deterministic_tie_keeps_first_candidate(self):
        lower, upper, reference = np.full(6, -2.), np.full(6, 2.), np.zeros(6)
        first, second = np.full(6, .1), np.full(6, -.1)
        np.testing.assert_array_equal(select_ik_candidate([first, second], reference, lower, upper), first)

    def test_invalid_inputs_are_rejected_even_with_empty_candidate_collection(self):
        for reference in ([0.], [float('inf')]*6):
            with self.assertRaises(ValueError):
                make_ik_seeds(reference, self.lower, self.upper)
        for margin in (-.1, float('nan'), True, 10.):
            with self.assertRaises(ValueError):
                self.seeds(margin=margin)
        for maximum in (0, -1, 1.5, True):
            with self.assertRaises(ValueError):
                self.seeds(max_seeds=maximum)
        with self.assertRaises(ValueError):
            select_ik_candidate([], self.reference, self.upper, self.lower)


if __name__ == '__main__':
    unittest.main()
