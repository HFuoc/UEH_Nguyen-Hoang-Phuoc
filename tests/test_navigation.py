import math
import sys
import unittest
from pathlib import Path
import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.navigation import scan_points, swept_clearance, enclosed_corridor, steering_curvature
from crc_solution.freshness import SensorFreshness
from crc_solution.perception import Perception
from crc_solution.control import Controller


class NavigationTests(unittest.TestCase):
    def test_continuous_edge_recovers_from_wrong_side_of_boundary(self):
        # bus_arc_v1 / 20261001T154851_633029Z / 61.70 s. The old row fit
        # relabelled the outer boundary and drove toward the sign pole.
        lane,_=Perception().lane(cv2.imread(str(Path(__file__).parent/'data/shoulder_boundary.jpg')))
        self.assertGreater(lane.confidence,.7)
        self.assertGreater(steering_curvature(lane),1.)

    def test_continuous_edge_does_not_reverse_supported_lane_heading(self):
        # full_v14 / 213.80 s: a second continuous line belongs to the other
        # lane; its trace must not reverse a strongly supported right heading.
        lane,_=Perception().lane(cv2.imread(str(Path(__file__).parent/'data/angled_boundary.jpg')))
        self.assertGreater(lane.confidence,.7)
        self.assertLess(steering_curvature(lane),-.5)

    def test_paired_curve_does_not_follow_wrong_straight_boundary(self):
        # full_v12 / 20261001T130748_381014Z / 231.30 s. A straight fit
        # steered left into the outside of a clearly right-curving corridor.
        lane,_=Perception().lane(cv2.imread(str(Path(__file__).parent/'data/curve_boundary.jpg')))
        self.assertGreater(lane.confidence,.5)
        self.assertLess(steering_curvature(lane),-.5)

    def test_only_extended_paired_walls_shorten_lookahead(self):
        x=np.linspace(-.2,.7,40)
        walls=np.r_[np.column_stack((x,np.full(40,-.2))),np.column_stack((x,np.full(40,.5)))]
        self.assertTrue(enclosed_corridor(walls))
        self.assertFalse(enclosed_corridor(walls[:40]))
        self.assertFalse(enclosed_corridor(np.array([[.4,.2],[.5,-.2]])))
        self.assertFalse(enclosed_corridor(None))

    def test_sparse_boundary_after_hairpin_allows_slow_recovery(self):
        lane,_=Perception().lane(cv2.imread(str(Path(__file__).parent/'data/sparse_boundary.jpg')))
        speed,turn=Controller().step(0.,0.,0.,0.,lane,math.inf,True)
        self.assertGreater(speed,0.)
        self.assertLess(speed,.05)
        self.assertLess(turn,0.)

    def test_recorded_tunnel_posts_do_not_reverse_steering(self):
        lane, _ = Perception().lane(cv2.imread(str(Path(__file__).parent/'data/tunnel_bend.jpg')))
        self.assertGreater(lane.confidence, .5)
        self.assertGreater(lane.target_y, .05)
        self.assertGreater(lane.heading, 0.)

    def test_straight_obstacle_and_invalid_scan(self):
        self.assertAlmostEqual(swept_clearance(np.array([[.35, 0.]]), 0.), .36)
        self.assertTrue(math.isnan(swept_clearance(None, 0.)))
        self.assertTrue(math.isinf(swept_clearance(np.empty((0, 2)), 0.)))
        self.assertIsNone(scan_points([math.nan]*360, 0., math.pi/180, .12, 3.5))

    def test_curved_wall_does_not_block_safe_arc(self):
        # Two concentric walls with 0.20 m clearance from a left-turning path.
        theta = np.linspace(-.3, 1.6, 150)
        walls = np.concatenate([np.column_stack((r*np.sin(theta), .7-r*np.cos(theta)))
                                for r in (.5, .9)])
        self.assertLess(swept_clearance(walls, 0.), .6)
        self.assertGreater(swept_clearance(walls, 1/.7), .65)

    def test_obstacle_on_turn_and_tail_sweep_are_detected(self):
        angle=.3/.7
        obstacle=np.array([[.7*math.sin(angle), .7*(1-math.cos(angle))]])
        self.assertLess(swept_clearance(obstacle, 1/.7), .35)
        # Turning swings the rear right corner toward this point.
        self.assertLess(swept_clearance(np.array([[-.12, -.15]]), 3.), .3)

    def test_ramp_self_returns_keep_front_and_rear_obstacles(self):
        # A recorded ramp failure had a noisy self return at (0.075,-0.141).
        # Build each ray in a complete scan to check the actual input filter.
        for x,y,kept in ((.075,-.141,False),(-.05,.15,False),
                         (.12,.0,True),(-.12,-.15,True),(.06,-.18,True)):
            with self.subTest(point=(x,y)):
                angle=math.atan2(y,x+.064)
                ranges=[math.inf]*360
                ranges[180]=math.hypot(x+.064,y)
                points=scan_points(ranges,angle-math.pi,math.pi/180,.12,3.5)
                self.assertEqual(len(points),int(kept))


class FreshnessTests(unittest.TestCase):
    def test_duplicates_and_old_delivery_are_not_fresh(self):
        f=SensorFreshness()
        for name in ('image','scan','odom'):
            self.assertTrue(f.accept(name, 1., 10.))
        self.assertTrue(f.fresh(1.1,10.1,.8))
        self.assertFalse(f.accept('image',1.,10.7))
        self.assertFalse(f.accept('image',.9,10.7))
        self.assertFalse(f.fresh(1.9,10.9,.8))
        for name in ('image','scan','odom'): f.accept(name,2.,12.)
        self.assertFalse(f.fresh(5.,12.1,.8))

    def test_reset_and_future_stamps(self):
        f=SensorFreshness()
        for name in ('image','scan','odom'): f.accept(name,20.,1.)
        self.assertFalse(f.fresh(1.,1.1,.8))
        f.clear()
        for name in ('image','scan','odom'): f.accept(name,.1,2.)
        self.assertTrue(f.fresh(.2,2.1,.8))

    def test_future_outlier_does_not_poison_next_valid_sample(self):
        f=SensorFreshness()
        for name in ('image','scan','odom'):
            self.assertTrue(f.accept(name,1.,10.,now=1.))
            self.assertFalse(f.accept(name,100000.,10.1,now=1.1))
            self.assertEqual(f.stamps[name],1.)
            self.assertEqual(f.received[name],10.)
        # Rejected samples cannot keep the previous observations fresh.
        self.assertFalse(f.fresh(1.9,10.9,.8))
        for name in ('image','scan','odom'):
            self.assertTrue(f.accept(name,2.,11.,now=2.))
        self.assertTrue(f.fresh(2.1,11.1,.8))

    def test_small_clock_delivery_skew_is_accepted(self):
        f=SensorFreshness()
        self.assertTrue(f.accept('image',1.15,10.,now=1.))
        self.assertFalse(f.accept('image',1.3,10.1,now=1.))
        self.assertFalse(f.accept('image',1.2,10.1,now=math.nan))
        self.assertEqual(f.stamps['image'],1.15)


if __name__=='__main__': unittest.main()
