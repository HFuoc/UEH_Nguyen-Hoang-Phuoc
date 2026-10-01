"""Traffic image regressions: lighting does not change STOP into a lamp."""
import math
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.perception import Perception


class TrafficPerceptionTests(unittest.TestCase):
    def stop_image(self, background):
        image = np.full((480, 640, 3), background, np.uint8)
        octagon = np.array([(int(420+22*math.cos(angle)),
                             int(130+22*math.sin(angle)))
                            for angle in np.arange(8)*math.pi/4+math.pi/8])
        cv2.fillPoly(image, [octagon], (0, 0, 230))
        return image

    def test_stop_on_bright_background(self):
        signs = Perception().signs(self.stop_image(160))
        self.assertTrue(signs.stop)
        self.assertEqual(signs.light, 'unknown')

    def test_stop_on_dark_background_is_not_a_red_lamp(self):
        signs = Perception().signs(self.stop_image(20))
        self.assertTrue(signs.stop)
        self.assertEqual(signs.light, 'unknown')

    def test_housed_lamps_on_dim_background(self):
        for label, bgr in [('red', (0, 0, 255)), ('yellow', (0, 255, 255)),
                           ('green', (0, 255, 0))]:
            with self.subTest(color=label):
                image = np.full((480, 640, 3), 20, np.uint8)
                cv2.rectangle(image, (400, 70), (425, 155), (10, 10, 10), -1)
                cv2.circle(image, (412, 85), 8, bgr, -1)
                signs = Perception().signs(image)
                self.assertEqual(signs.light, label)
                self.assertFalse(signs.stop)

    def test_real_desaturated_green_lamp_keeps_housing_detection(self):
        image = cv2.imread(str(Path(__file__).parent/'data/green_lamp_annotated.jpg'))
        self.assertEqual(Perception().signs(image).light, 'green')

    def test_clipped_stop_backing_cannot_be_a_lamp(self):
        # A plate crossing the camera's right edge loses its octagonal shape.
        for center in (635, 640, 645):
            with self.subTest(center=center):
                image = np.full((480, 640, 3), 160, np.uint8)
                cv2.rectangle(image, (600, 70), (675, 155), (10, 10, 10), -1)
                polygon = np.array([(int(center+26*math.cos(a)), int(110+26*math.sin(a)))
                                    for a in np.arange(8)*math.pi/4+math.pi/8])
                cv2.fillPoly(image, [polygon], (0, 0, 230))
                self.assertEqual(Perception().signs(image).light, 'unknown')


if __name__ == '__main__':
    unittest.main()
