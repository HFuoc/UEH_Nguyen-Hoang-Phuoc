"""Traffic state regressions independent of ROS and a particular track."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.control import Controller
from crc_solution.perception import Lane, Signs


class TrafficTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.lane = Lane(confidence=1.)

    def observe(self, signs, time, distance=0.):
        for offset in (0., .1, .2):
            self.controller.observe(signs, time+offset, distance)

    def step(self, time, distance=0.):
        return self.controller.step(time, distance, 0., 0., self.lane,
                                    math.inf, True)

    def finish_stop(self):
        self.observe(Signs(stop=True, stop_distance=.4), 0.)
        self.step(.2)
        self.controller.observe(Signs(stop=True, stop_distance=.4), 2.4, 0.)
        self.step(2.5)
        self.assertFalse(self.controller.stop_pending)

    def test_continuously_visible_stop_does_not_rearm_after_travel(self):
        self.finish_stop()
        self.observe(Signs(stop=True, stop_distance=.3), 2.6, .81)
        self.assertFalse(self.controller.stop_pending)
        self.assertGreater(self.step(2.9, .81)[0], 0.)

    def test_stop_absence_without_travel_does_not_rearm(self):
        self.finish_stop()
        self.controller.observe(Signs(), 5., .2)
        self.observe(Signs(stop=True, stop_distance=.3), 5.1, .2)
        self.assertFalse(self.controller.stop_pending)

    def test_new_stop_after_absence_and_travel_is_handled(self):
        self.finish_stop()
        self.controller.observe(Signs(), 4., .81)
        self.observe(Signs(stop=True, stop_distance=.4), 4.1, .81)
        self.assertTrue(self.controller.stop_pending)
        self.assertEqual(self.step(4.4, .81), (0., 0.))

    def test_yellow_still_stops_after_nearby_green_without_entry(self):
        self.observe(Signs(light='green', light_distance=.29), 0.)
        self.assertGreater(self.step(.2)[0], 0.)
        self.observe(Signs(light='yellow', light_distance=.28), .3, .01)
        self.assertEqual(self.step(.5, .01), (0., 0.))
        self.assertEqual(self.controller.state, 'LIGHT_WAIT')

    def test_stationary_green_does_not_mean_inside_junction(self):
        for index in range(4):
            self.observe(Signs(light='green', light_distance=.2), index*.3)
            self.step(index*.3+.2)
        self.observe(Signs(light='red', light_distance=.2), 1.3)
        self.assertEqual(self.step(1.5), (0., 0.))

    def test_yellow_after_reaching_entry_does_not_stop_inside(self):
        self.observe(Signs(light='green', light_distance=.29), 0.)
        self.step(.2)
        self.step(.3, .27)
        self.observe(Signs(light='yellow', light_distance=.1), .4, .28)
        self.assertGreater(self.step(.6, .28)[0], 0.)

    def test_green_frames_cannot_extend_committed_junction(self):
        self.observe(Signs(light='green', light_distance=.29), 0.)
        self.step(.2)
        self.step(.3, .27)
        end = self.controller.junction_until
        self.observe(Signs(light='green', light_distance=.1), .4, .4)
        self.step(.6, .4)
        self.assertEqual(self.controller.junction_until, end)
        self.observe(Signs(light='red', light_distance=.4), 1., end+.01)
        self.assertEqual(self.step(1.2, end+.01), (0., 0.))

    def test_stale_entry_and_distant_green_cannot_commit_junction(self):
        self.observe(Signs(light='green', light_distance=.29), 0.)
        self.step(.2)
        self.observe(Signs(light='green', light_distance=2.16), 60., .27)
        self.step(60.2, .27)
        self.assertLess(self.controller.junction_until, .27)
        self.observe(Signs(light='red', light_distance=.2), 60.3, .28)
        self.assertEqual(self.step(60.5, .28), (0., 0.))

    def test_wait_requires_confirmed_nearby_green_to_release(self):
        self.observe(Signs(light='red', light_distance=.4), 0.)
        self.assertEqual(self.step(.2), (0., 0.))
        self.controller.observe(Signs(), 2., 0.)
        self.assertEqual(self.step(2.), (0., 0.))
        self.observe(Signs(light='green', light_distance=2.16), 3.)
        self.assertEqual(self.step(3.2), (0., 0.))
        self.controller.observe(Signs(), 5., 0.)
        for time in (5.1, 5.2):
            self.controller.observe(Signs(light='green', light_distance=.4), time, 0.)
            self.assertEqual(self.step(time), (0., 0.))
        self.controller.observe(Signs(light='green', light_distance=.4), 5.3, 0.)
        self.assertGreater(self.step(5.3)[0], 0.)
        self.assertLess(self.controller.junction_until, 0.)


if __name__ == '__main__':
    unittest.main()
