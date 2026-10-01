"""Template-derived regression tests, not evidence of novel-track accuracy."""
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/crc_solution'))
from crc_solution.sign_classifier import SignClassifier


class SignClassifierTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.classifier = SignClassifier()
        directory = Path(__file__).resolve().parents[1]/'src/crc_solution/crc_solution/assets'
        cls.artwork = {label: cv2.imread(str(directory/f'sign_{label}.png'))
                       for label in cls.classifier.labels}

    def frame(self, label, size=64, gain=1., noise=0., perspective=False):
        patch = cv2.resize(self.artwork[label], (size, size), interpolation=cv2.INTER_AREA)
        if perspective:
            # Held-out corner offsets, distinct from the startup view bank.
            corners = np.float32([[.051, .039], [.918, .018], [.978, .944], [.011, .980]])
            source = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
            transform = cv2.getPerspectiveTransform(source*(size-1), corners*(size-1))
            patch = cv2.warpPerspective(patch, transform, (size, size),
                                       borderValue=(30, 30, 30))
        rng = np.random.default_rng(37)
        patch = np.clip(patch*gain+rng.normal(0, noise, patch.shape), 0, 255).astype(np.uint8)
        image = np.full((480, 640, 3), 140, np.uint8)
        image[100:100+size, 400:400+size] = patch
        return image

    def assert_only_label(self, image, label):
        observed = self.classifier.detect(image)
        self.assertEqual([item.label for item in observed], [label])
        self.assertGreaterEqual(observed[0].confidence, self.classifier.min_confidence)
        self.assertGreater(observed[0].distance, 0.)
        self.assertEqual(len(observed[0].bbox), 4)

    def test_all_eight_resized_appearances(self):
        for label in self.classifier.labels:
            with self.subTest(label=label):
                self.assert_only_label(self.frame(label), label)

    def test_small_dim_noisy_appearances(self):
        for label in self.classifier.labels:
            with self.subTest(label=label):
                self.assert_only_label(self.frame(label, size=28, gain=.45, noise=2.), label)

    def test_mild_unseen_perspective_and_brightness(self):
        for label in self.classifier.labels:
            with self.subTest(label=label):
                self.assert_only_label(self.frame(label, size=46, gain=.7,
                                                  noise=2., perspective=True), label)

    def test_no_guess_when_icon_is_occluded(self):
        for label in self.classifier.labels:
            image = self.frame(label)
            image[117:143, 414:450] = 30
            with self.subTest(label=label):
                self.assertTrue(all(item.label == label for item in self.classifier.detect(image)))

    def test_rejects_unmarked_shapes_and_traffic_lamps(self):
        negatives = []
        for color in ((170, 60, 25), (0, 0, 230), (50, 150, 25)):
            image = np.full((480, 640, 3), 120, np.uint8)
            cv2.rectangle(image, (400, 100), (460, 160), color, -1)
            negatives.append(image)
        image = np.full((480, 640, 3), 120, np.uint8)
        cv2.fillPoly(image, [np.array([[420, 100], [390, 155], [450, 155]])], (0, 0, 230))
        cv2.fillPoly(image, [np.array([[420, 110], [400, 150], [440, 150]])], (240, 240, 240))
        negatives.append(image)
        for color in ((0, 0, 255), (0, 255, 0)):
            image = np.full((480, 640, 3), 120, np.uint8)
            cv2.rectangle(image, (400, 70), (425, 155), (10, 10, 10), -1)
            cv2.circle(image, (412, 85), 8, color, -1)
            negatives.append(image)
        for index, image in enumerate(negatives):
            with self.subTest(negative=index):
                self.assertEqual(self.classifier.detect(image), [])

    def test_tiny_or_random_regions_do_not_get_a_label(self):
        self.assertEqual(self.classifier.detect(self.frame('stop', size=8)), [])
        noise = np.random.default_rng(94).integers(0, 256, (480, 640, 3), dtype=np.uint8)
        self.assertEqual(self.classifier.detect(noise), [])

    def test_depth_scales_with_observed_size_and_camera_focal_length(self):
        far = self.classifier.detect(self.frame('stop', size=32))[0]
        near = self.classifier.detect(self.frame('stop', size=64))[0]
        self.assertAlmostEqual(far.distance/near.distance, 2., delta=.10)
        original = self.classifier.fx
        try:
            self.classifier.fx = original*1.5
            recalibrated = self.classifier.detect(self.frame('stop', size=64))[0]
            self.assertAlmostEqual(recalibrated.distance/near.distance, 1.5)
        finally:
            self.classifier.fx = original


if __name__ == '__main__':
    unittest.main()
