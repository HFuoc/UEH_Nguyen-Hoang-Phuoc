"""Conservative recognition of the eight supplied traffic-sign appearances.

Templates are packaged locally: runtime only reads ``crc_solution/assets``.
No map, pose, simulator model, or simulator service is used. These are appearance
templates, so the classifier intentionally abstains on unfamiliar/ambiguous icons.
Perspective-template tests establish regression coverage, not track generalization.
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectedSign:
    label: str
    bbox: tuple
    confidence: float
    distance: float


def _colors(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    chromatic = (sat > 65) & (val > 28)
    red = chromatic & ((hue < 14) | (hue > 168))
    blue = chromatic & (hue > 93) & (hue < 135)
    green = chromatic & (hue > 36) & (hue < 92)
    return np.stack((red, blue, green), axis=-1).astype(np.float32)


def _unit(values):
    values = values.astype(np.float32).reshape(-1)
    values -= float(values.mean())
    return values / max(float(np.linalg.norm(values)), 1e-6)


def _features(patch):
    canonical = cv2.resize(patch, (48, 48), interpolation=cv2.INTER_AREA)
    canonical = cv2.GaussianBlur(canonical, (3, 3), .65)
    gray = cv2.cvtColor(canonical, cv2.COLOR_BGR2GRAY).astype(np.float32)
    # Mean-centering removes additive brightness, unit length removes gain.
    # The icon carries more weight than the common coloured border/shape.
    return (_unit(gray), _unit(gray[18:40, 10:38]),
            _unit(cv2.resize(_colors(canonical), (16, 16), interpolation=cv2.INTER_AREA)))


def _template_views(image):
    """Small appearance bank for mild tilt/keystone; generated once at startup."""
    size = image.shape[0]
    square = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    views = [image]
    for base in (np.float32([[.06, .04], [.90, 0], [1, .90], [0, 1]]),
                 np.float32([[0, .10], [1, 0], [.96, 1], [.05, .91]])):
        for flip in (False, True):
            corners = base.copy()
            if flip:
                corners[:, 0] = 1-corners[:, 0]
                corners = corners[[1, 0, 3, 2]]
            for vertical_flip in (False, True):
                target = corners.copy()
                if vertical_flip:
                    target[:, 1] = 1-target[:, 1]
                    target = target[[3, 2, 1, 0]]
                transform = cv2.getPerspectiveTransform(square*(size-1), target*(size-1))
                views.append(cv2.warpPerspective(image, transform, (size, size),
                                                 borderValue=(30, 30, 30)))
    for angle in (-6., 6.):
        transform = cv2.getRotationMatrix2D(((size-1)/2, (size-1)/2), angle, .88)
        views.append(cv2.warpAffine(image, transform, (size, size),
                                   borderValue=(30, 30, 30)))
    features = []
    for view in views:
        mask = np.any(_colors(view) > 0, axis=2).astype(np.uint8)
        x, y, width, height = cv2.boundingRect(mask)
        features.append(_features(view[y:y+height, x:x+width]))
    return features


class SignClassifier:
    labels = ('stop', 'crosswalk', 'bus', 'hw_entry', 'hw_exit',
              'ramp', 'tunnel', 'uneven')

    def __init__(self, fx=381.36, min_confidence=.76, min_margin=.065):
        self.fx = float(fx)
        self.min_confidence = float(min_confidence)
        self.min_margin = float(min_margin)
        self.templates = []
        directory = Path(__file__).resolve().parent/'assets'
        for label in self.labels:
            path = directory/f'sign_{label}.png'
            image = cv2.imread(str(path))
            if image is None:
                raise FileNotFoundError(f'Missing packaged sign template: {path}')
            mask = np.any(_colors(image) > 0, axis=2).astype(np.uint8)
            x, y, width, height = cv2.boundingRect(mask)
            self.templates.append((label, _template_views(image), width/image.shape[1]))

    def detect(self, image):
        """Return confident DetectedSign objects from a uint8 BGR frame.

        bbox=(x,y,width,height) bounds the coloured sign face. ``distance`` is
        an approximate camera depth in metres using a 70 mm printed plate and
        the caller's camera ``fx``. It is not a junction/stop-line distance.
        """
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            return []
        colors = _colors(image)
        mask = np.any(colors > 0, axis=2).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = image.shape[:2]
        detections = []
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            if (min(bw, bh) < 12 or bw*bh > .18*width*height
                    or not .42 < bw/bh < 1.8 or cv2.contourArea(contour) < .20*bw*bh):
                continue
            patch = image[y:y+bh, x:x+bw]
            features = _features(patch)
            scores = []
            for label, views, colored_width in self.templates:
                score = max(sum(weight*float(np.dot(a, b)) for weight, a, b in
                                zip((.30, .55, .15), features, template))
                            for template in views)
                scores.append((score, label, colored_width))
            scores.sort(reverse=True)
            best, label, colored_width = scores[0]
            if best < self.min_confidence or best-scores[1][0] < self.min_margin:
                continue
            detections.append(DetectedSign(label, (x, y, bw, bh), float(best),
                                           self.fx*.070*colored_width/bw))
        return sorted(detections, key=lambda item: item.distance)
