import math
import sys
from pathlib import Path
import unittest
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.perception import Lane, Signs, Perception, corridor_range
from crc_solution.control import Controller, Settings


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller()
        self.lane = Lane(0.,0.,1.,0.)

    def step(self, t, front=math.inf, fresh=True, speed=0., lane=None, distance=0.):
        return self.c.step(t,distance,0.,speed,lane or self.lane,front,fresh)

    def test_waits_for_all_sensors(self):
        self.assertEqual(self.step(0,fresh=False),(0.,0.))
        self.assertGreater(self.step(1)[0],0.)
        self.assertEqual(self.step(2,fresh=False),(0.,0.))

    def test_obstacle_requires_clear_dwell(self):
        self.assertEqual(self.step(0,front=.22),(0.,0.))
        self.assertEqual(self.step(.1),(0.,0.))
        self.assertEqual(self.step(.5),(0.,0.))
        self.assertGreater(self.step(.8)[0],0.)

    def test_nan_scan_is_not_clear(self):
        self.assertEqual(self.step(0,front=math.nan),(0.,0.))

    def test_stop_requires_stationary_two_seconds(self):
        for t in (0.,.1,.2):
            self.c.observe(Signs(stop=True,stop_distance=.4),t,0.)
        self.assertEqual(self.step(.2,speed=.1),(0.,0.))
        self.step(.5)
        self.assertEqual(self.step(2.4),(0.,0.))
        self.step(2.8)
        self.assertGreater(self.step(2.9)[0],0.)
        for t in (3.,3.1,3.2):
            self.c.observe(Signs(stop=True,stop_distance=.3),t,.1)
        self.assertFalse(self.c.stop_pending)

    def test_red_wait_and_green_release(self):
        for t in (0.,.1,.2):
            self.c.observe(Signs(light='red',light_distance=.4),t,0.)
        self.assertEqual(self.step(.2),(0.,0.))
        self.assertEqual(self.step(3),(0.,0.))
        for t in (3.,3.1,3.2):
            self.c.observe(Signs(light='green',light_distance=.4),t,0.)
        self.assertGreater(self.step(3.2)[0],0.)

    def test_yellow_before_and_inside_junction(self):
        for t in (0.,.1,.2):
            self.c.observe(Signs(light='green',light_distance=.25),t,0.)
        self.step(.2)
        for t in (.3,.4,.5):
            self.c.observe(Signs(light='yellow',light_distance=.2),t,.1)
        self.assertGreater(self.step(.5,distance=.1)[0],0.)
        self.c = Controller()
        for t in (0.,.1,.2):
            self.c.observe(Signs(light='yellow',light_distance=.4),t,0.)
        self.assertEqual(self.step(.2),(0.,0.))

    def test_lost_lane_is_bounded(self):
        self.step(0)
        self.assertGreater(self.step(.2,lane=Lane())[0],0.)
        self.assertEqual(self.step(.6,lane=Lane()),(0.,0.))

    def test_turn_direction_and_speed_parameter(self):
        self.assertGreater(self.step(0,lane=Lane(.05,.1,1.,.06))[1],0.)
        self.c.settings.max_speed=.06
        self.assertLessEqual(self.step(.1)[0],.06)

    def test_lidar_angle_origin(self):
        rays = [math.inf]*360
        rays[180] = .30
        self.assertAlmostEqual(corridor_range(rays,-math.pi,math.pi/180,.12,3.5),.236)
        self.assertTrue(math.isnan(corridor_range([math.nan]*360,0,math.pi/180,.12,3.5)))
        self.assertEqual(corridor_range([math.inf]*360,0,math.pi/180,.12,3.5),math.inf)

    def test_rejects_unsafe_settings(self):
        for kwargs in ({'stop_hold':1.9},{'max_speed':math.nan},{'max_speed':-.1}):
            with self.assertRaises(ValueError):
                Settings(**kwargs).validate()

    def test_clock_reset_drops_old_stop_timer(self):
        for t in (10.,10.1,10.2):
            self.c.observe(Signs(stop=True,stop_distance=.4),t,0.)
        self.step(10.3)
        self.step(.1, fresh=False)
        self.assertFalse(self.c.stop_pending)


class ImageTests(unittest.TestCase):
    def test_real_unmarked_ramp_edge(self):
        image=cv2.imread(str(Path(__file__).parent/'data/ramp_camera.jpg'))
        lane,_=Perception().lane(image)
        self.assertGreater(lane.confidence,.35)
        self.assertLess(abs(lane.target_y),.06)

    def test_real_desaturated_green_lamp(self):
        image=cv2.imread(str(Path(__file__).parent/'data/green_lamp_annotated.jpg'))
        signs=Perception().signs(image)
        self.assertEqual(signs.light,'green')

    def test_real_taper_frame_keeps_near_corridor(self):
        # Captured from the first Gazebo trial; annotation is retained so its
        # provenance is visible. This is a detection regression, not truth RMS.
        image=cv2.imread(str(Path(__file__).parent/'data/taper_annotated.jpg'))
        lane,_=Perception().lane(image)
        self.assertGreater(lane.confidence,.5)
        self.assertLess(abs(lane.target_y),.04)

    def lane_image(self, shift=0., brightness=230):
        p=Perception()
        img=np.full((480,640,3),20,dtype=np.uint8)
        for side in (-.175,.175):
            pts=[]
            for row in range(265,479):
                z=p.height*p.fy/(row-p.cy)
                pts.append((int(p.cx-(side+shift)*p.fx/z),row))
            cv2.polylines(img,[np.array(pts)],False,(brightness,)*3,4)
        return img

    def test_projected_lane_center_and_offset(self):
        for shift in (0.,.035,-.035):
            lane,_=Perception().lane(self.lane_image(shift))
            self.assertGreater(lane.confidence,.35)
            self.assertAlmostEqual(lane.lateral,shift,delta=.012)

    def test_dark_lane_and_blank(self):
        lane,_=Perception().lane(self.lane_image(brightness=65))
        self.assertGreater(lane.confidence,.35)
        lane,_=Perception().lane(np.full((480,640,3),20,np.uint8))
        self.assertLess(lane.confidence,.35)

    def test_warning_triangle_is_not_stop(self):
        img=np.full((480,640,3),160,np.uint8)
        cv2.fillPoly(img,[np.array([[400,100],[370,155],[430,155]])],(0,0,230))
        self.assertFalse(Perception().signs(img).stop)

    def test_stop_plate_and_lamp(self):
        img=np.full((480,640,3),160,np.uint8)
        pts=np.array([(int(420+22*math.cos(a)),int(130+22*math.sin(a)))
                      for a in np.arange(8)*math.pi/4+math.pi/8])
        cv2.fillPoly(img,[pts],(0,0,230))
        self.assertTrue(Perception().signs(img).stop)
        img=np.full((480,640,3),160,np.uint8)
        cv2.rectangle(img,(400,70),(425,155),(10,10,10),-1)
        cv2.circle(img,(412,85),8,(0,0,255),-1)
        signs=Perception().signs(img)
        self.assertEqual(signs.light,'red')
        self.assertFalse(signs.stop)


if __name__=='__main__':
    unittest.main()
