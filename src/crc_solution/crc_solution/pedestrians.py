"""Lightweight supplied-pedestrian appearance and crossing occupancy tracking.

The purple/indigo clothing cue is specific to the supplied simulated actor;
generic LiDAR collision stopping remains active for all other objects.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass
class Person:
    forward: float
    lateral: float
    radius: float = .03
    box: tuple = ()


def detect_people(image, camera):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (118, 65, 30), (165, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result=[]
    for contour in contours:
        x,y,w,h=cv2.boundingRect(contour)
        if h < 12 or w < 3 or not 1.6 < h/w < 7. or cv2.contourArea(contour) < .3*w*h:
            continue
        if y+h <= camera.cy+6:
            continue
        depth=camera.height*camera.fy/(y+h-camera.cy)
        if not .12 < depth < 2.5 or not .025 < w*depth/camera.fx < .10:
            continue
        lateral=-(x+w/2-camera.cx)*depth/camera.fx
        result.append(Person(depth+camera.offset,lateral,.03,(x,y,w,h)))
    return result


class CrossingGuard:
    def __init__(self):
        self.waiting=False
        self.clear_since=None
        self.last_person=None
        self.last_seen=-math.inf
        self.last_time=-math.inf

    def update(self, people, points, lane, now):
        if now < self.last_time:
            self.__init__()
        self.last_time=now
        occupied=False
        for person in people:
            # Right-hand travel: monitor both our lane and the lane to the
            # left, including the actor's radius at either road boundary.
            centre = lane.lateral + (person.forward-.30)*math.tan(lane.heading)
            relative = person.lateral-centre
            if .10 < person.forward < 1.3 and -.175-person.radius < relative < .525+person.radius:
                occupied=True
                self.last_person=np.array([person.forward,person.lateral])
                self.last_seen=now
        # Bridge brief image occlusion with nearby LiDAR returns. Expire this
        # association so a static object cannot masquerade as a person forever.
        if (not occupied and self.waiting and self.last_person is not None
                and now-self.last_seen < 1.0 and points is not None and len(points)):
            occupied=bool(np.any(np.linalg.norm(points-self.last_person,axis=1) < .12))
        if occupied:
            self.waiting=True
            self.clear_since=None
        elif self.waiting:
            if self.clear_since is None:
                self.clear_since=now
            elif now-self.clear_since >= 1.0:
                self.waiting=False
        return self.waiting
