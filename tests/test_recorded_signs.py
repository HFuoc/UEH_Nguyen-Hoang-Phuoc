"""Regression on onboard images, separate from augmented template tests."""
import sys
import unittest
from pathlib import Path
import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.sign_classifier import SignClassifier
from crc_solution.pedestrians import detect_people
from crc_solution.perception import Perception


class RecordedSignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.classifier=SignClassifier()

    def test_known_visible_signs_in_camera_frames(self):
        for label in ('stop','crosswalk','ramp','tunnel'):
            with self.subTest(label=label):
                frame=cv2.imread(str(Path(__file__).parent/'data'/f'sign_{label}_real.jpg'))
                labels=[d.label for d in self.classifier.detect(frame)]
                self.assertIn(label,labels)

    def test_pedestrian_seen_in_crosswalk_frame(self):
        frame=cv2.imread(str(Path(__file__).parent/'data/sign_crosswalk_real.jpg'))
        people=detect_people(frame,Perception())
        self.assertEqual(len(people),1)
        self.assertGreater(people[0].lateral,0.)
        self.assertTrue(1. < people[0].forward < 2.)


if __name__=='__main__': unittest.main()
