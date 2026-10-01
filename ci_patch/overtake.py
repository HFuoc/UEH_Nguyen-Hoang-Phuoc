"""Sensor-only highway overtaking using LiDAR and relative odometry.

The manoeuvre contains no track coordinates. It is armed by a confirmed
highway-entry sign and a persistent obstacle in the current lane, then uses
relative yaw/travel to make a bounded S-shaped lane change, pass, and return.
"""
import math
import numpy as np

from .navigation import swept_clearance


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def front_object_distance(points):
    """Robust distance to a broad object centred in the current lane."""
    if points is None or len(points) < 3:
        return math.inf
    p = points[(points[:, 0] > .12) & (points[:, 0] < 1.15)
               & (np.abs(points[:, 1]) < .16)]
    if len(p) < 3:
        return math.inf
    xs = np.sort(p[:, 0])
    x0 = xs[min(1, len(xs)-1)]
    cluster = p[np.abs(p[:, 0]-x0) < .08]
    if len(cluster) < 3:
        return math.inf
    return float(np.median(cluster[:, 0]))


def passing_side_open(points):
    """Reject a lane change when the immediate lane to the left is occupied."""
    if points is None:
        return False
    zone = points[(points[:, 0] > -.05) & (points[:, 0] < .80)
                  & (points[:, 1] > .16) & (points[:, 1] < .39)]
    # A few isolated returns are tolerated; a vehicle/wall is a dense cluster.
    return len(zone) < 5


class OvertakeManager:
    TURN = 2.2
    TARGET_YAW = .90

    def __init__(self):
        self.state = 'IDLE'
        self.anchor_yaw = 0.
        self.phase_distance = 0.
        self.detect_since = None
        self.last_time = -math.inf
        self.completed_distance = -math.inf

    @property
    def active(self):
        return self.state not in ('IDLE', 'DONE')

    def reset(self):
        self.__init__()

    def _begin(self, distance, yaw):
        self.state = 'SHIFT_LEFT'
        self.anchor_yaw = yaw
        self.phase_distance = distance
        self.detect_since = None

    def _advance(self, state, distance):
        self.state = state
        self.phase_distance = distance

    def command(self, highway, points, distance, yaw, now):
        """Return ``(v, w, state)`` while overtaking, otherwise ``None``."""
        if now < self.last_time:
            self.reset()
        self.last_time = now

        if self.state == 'DONE':
            if distance-self.completed_distance > .55 or not highway:
                self.state = 'IDLE'
            else:
                return None

        if self.state == 'IDLE':
            if not highway:
                self.detect_since = None
                return None
            obstacle = front_object_distance(points)
            ready = .35 < obstacle < 1.00 and passing_side_open(points)
            if not ready:
                self.detect_since = None
                return None
            if self.detect_since is None:
                self.detect_since = now
                return None
            if now-self.detect_since < .35:
                return None
            self._begin(distance, yaw)

        yaw_error = _angle(yaw-self.anchor_yaw)
        travelled = distance-self.phase_distance

        if self.state == 'SHIFT_LEFT':
            curvature = self.TURN
            if yaw_error >= self.TARGET_YAW:
                self._advance('ALIGN_PASS', distance)
                curvature = -self.TURN
        elif self.state == 'ALIGN_PASS':
            curvature = -self.TURN
            if yaw_error <= .06 and travelled > .20:
                self._advance('PASS', distance)
                curvature = 0.
        elif self.state == 'PASS':
            curvature = float(np.clip(-1.2*yaw_error, -.7, .7))
            if travelled >= .48:
                self._advance('RETURN_RIGHT', distance)
                curvature = -self.TURN
        elif self.state == 'RETURN_RIGHT':
            curvature = -self.TURN
            if yaw_error <= -self.TARGET_YAW:
                self._advance('ALIGN_HOME', distance)
                curvature = self.TURN
        elif self.state == 'ALIGN_HOME':
            curvature = self.TURN
            if yaw_error >= -.06 and travelled > .20:
                self.state = 'DONE'
                self.completed_distance = distance
                return None
        else:
            self.reset()
            return None

        # Immediate swept-footprint safety remains active during the pass.
        clearance = swept_clearance(points, curvature, horizon=.38)
        if math.isnan(clearance) or clearance < .18:
            return 0., 0., 'OVERTAKE_BLOCKED'
        speed = .075 if self.state != 'PASS' else .09
        turn = speed*curvature
        return speed, turn, 'OVERTAKE_'+self.state
