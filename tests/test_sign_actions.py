"""Temporal confirmation/rearm tests for the sign behaviour adapter."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.sign_actions import SignActions
from crc_solution.sign_classifier import DetectedSign


def sign(label, depth=.5):
    return DetectedSign(label, (100, 100, 40, 40), .9, depth)


class SignActionTests(unittest.TestCase):
    def setUp(self):
        self.actions = SignActions()

    def confirm(self, label, now=0., distance=0.):
        self.actions.update([sign(label)], now, distance)
        self.actions.update([sign(label)], now+.2, distance)

    def test_confirmation_requires_separate_recent_frames(self):
        self.actions.update([sign('ramp'), sign('ramp')], 0., 0.)
        self.assertEqual(self.actions.speed_cap(0.), math.inf)
        self.actions.update([sign('ramp')], .8, 0.)
        self.assertEqual(self.actions.speed_cap(0.), math.inf)
        self.actions.update([sign('ramp')], 1., 0.)
        self.assertEqual(self.actions.speed_cap(0.), .10)

    def test_missing_frame_resets_confirmation(self):
        self.actions.update([sign('tunnel')], 0., 0.)
        self.actions.update([], .1, 0.)
        self.actions.update([sign('tunnel')], .2, 0.)
        self.assertEqual(self.actions.speed_cap(0.), math.inf)
        self.actions.update([sign('tunnel')], .4, 0.)
        self.assertEqual(self.actions.speed_cap(0.), .10)

    def test_caution_signs_cap_speed_and_expire_by_travel(self):
        for label in ('ramp', 'tunnel', 'uneven', 'crosswalk', 'bus'):
            with self.subTest(label=label):
                self.actions = SignActions()
                self.confirm(label, distance=3.)
                self.assertEqual(self.actions.speed_cap(3.), .10)
                self.assertEqual(self.actions.speed_cap(5.49), .10)
                self.assertEqual(self.actions.speed_cap(5.5), math.inf)

    def test_continuously_visible_sign_does_not_extend_zone(self):
        self.confirm('ramp')
        for index in range(1, 9):
            self.actions.update([sign('ramp')], .2+index*.2, index*.2)
        self.assertEqual(self.actions.caution_until, 2.5)

    def test_same_type_rearms_after_absence_and_travel(self):
        self.confirm('ramp')
        self.actions.update([], .5, .3)
        self.actions.update([sign('ramp')], 1.4, 1.1)
        self.assertEqual(self.actions.caution_until, 2.5)
        self.actions.update([sign('ramp')], 1.6, 1.1)
        self.assertAlmostEqual(self.actions.caution_until, 3.6)

    def test_absence_without_travel_does_not_rearm(self):
        self.confirm('ramp')
        self.actions.update([], 2., .2)
        self.confirm('ramp', 2.2, .2)
        self.assertEqual(self.actions.caution_until, 2.5)

    def test_highway_entry_exit_and_later_entry(self):
        self.confirm('hw_entry')
        self.assertTrue(self.actions.highway)
        self.actions.update([], .4, .2)
        self.confirm('hw_exit', .6, .5)
        self.assertFalse(self.actions.highway)
        self.confirm('hw_entry', 2., 1.5)
        self.assertTrue(self.actions.highway)
        self.assertEqual(self.actions.speed_cap(1.5), math.inf)

    def test_clock_reset_clears_caution_highway_and_confirmation(self):
        self.confirm('ramp', 10., 2.)
        self.confirm('hw_entry', 11., 3.)
        self.actions.update([sign('hw_entry')], .1, 0.)
        self.assertFalse(self.actions.highway)
        self.assertEqual(self.actions.speed_cap(0.), math.inf)
        self.assertEqual(self.actions.handled, {})
        self.actions.update([sign('hw_entry')], .3, 0.)
        self.assertTrue(self.actions.highway)

    def test_far_or_invalid_depth_does_not_trigger(self):
        for depth in (1.2, 3., math.nan, math.inf, -.1, 0.):
            with self.subTest(depth=depth):
                actions = SignActions()
                actions.update([sign('ramp', depth)], 0., 0.)
                actions.update([sign('ramp', depth)], .2, 0.)
                self.assertEqual(actions.speed_cap(0.), math.inf)
                self.assertEqual(actions.label, 'unknown')


if __name__ == '__main__':
    unittest.main()
