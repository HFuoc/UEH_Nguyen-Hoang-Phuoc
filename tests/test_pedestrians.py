import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.pedestrians import Person, CrossingGuard, detect_people
from crc_solution.perception import Lane, Perception


class PedestrianTests(unittest.TestCase):
    def test_waits_in_other_lane_then_requires_clear_dwell(self):
        guard=CrossingGuard()
        lane=Lane(confidence=1.)
        self.assertTrue(guard.update([Person(.6,0.)],None,lane,0.))
        self.assertTrue(guard.update([Person(.6,.40)],None,lane,1.))
        self.assertTrue(guard.update([Person(.6,.64)],None,lane,2.))
        self.assertTrue(guard.update([],None,lane,2.9))
        self.assertFalse(guard.update([],None,lane,3.1))

    def test_kerb_and_distant_actor_do_not_stop(self):
        guard=CrossingGuard()
        self.assertFalse(guard.update([Person(.6,-.28),Person(.6,.64),Person(2.,0.)],None,Lane(),0.))

    def test_short_camera_occlusion_uses_scan(self):
        g=CrossingGuard(); lane=Lane()
        g.update([Person(.6,.3)],None,lane,0.)
        self.assertTrue(g.update([],np.array([[.61,.32]]),lane,.8))
        self.assertTrue(g.update([],np.empty((0,2)),lane,1.))
        self.assertFalse(g.update([],np.empty((0,2)),lane,2.1))

    def test_projected_actor_and_coloured_square_negative(self):
        p=Perception(); img=np.full((480,640,3),30,np.uint8)
        cv2.rectangle(img,(380,193),(407,315),(180,0,180),-1)
        people=detect_people(img,p)
        self.assertEqual(len(people),1)
        self.assertAlmostEqual(people[0].forward,.66,delta=.04)
        self.assertLess(people[0].lateral,0.)
        img[:]=30
        cv2.rectangle(img,(380,285),(410,315),(180,0,180),-1)
        self.assertEqual(detect_people(img,p),[])


if __name__=='__main__': unittest.main()
