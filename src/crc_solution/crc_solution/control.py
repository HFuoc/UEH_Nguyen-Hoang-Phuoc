"""Deterministic behaviour arbitration; independent of ROS for replay/tests."""
from dataclasses import dataclass
import math
from .perception import Lane, Signs


@dataclass
class Settings:
    max_speed: float = .12
    max_turn: float = 1.0
    steering_gain: float = 1.0
    stop_distance: float = .30
    sensor_timeout: float = .8
    lane_grace: float = .5
    stop_hold: float = 2.2

    def validate(self):
        for key, (lo, hi) in PARAMETER_LIMITS.items():
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f'{key}: range {lo}..{hi}')


PARAMETER_LIMITS = {'max_speed':(0., .30), 'max_turn':(.05, 1.8), 'steering_gain':(.1, 3.),
                    'stop_distance':(.20, .8), 'sensor_timeout':(.1, 3.),
                    'lane_grace':(0., .8), 'stop_hold':(2., 5.)}


class Controller:
    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.state = 'WAIT_SENSORS'
        self.last_time = None
        self.last_lane_time = -math.inf
        self.last_lane = Lane()
        self.last_yaw = 0.
        self.stop_count = 0
        self.stop_pending = False
        self.stop_stationary_since = None
        self.stop_completed_at = -math.inf
        self.stop_seen_at = -math.inf
        self.light_color = 'unknown'
        self.light_candidate = 'unknown'
        self.light_count = 0
        self.light_distance = math.inf
        self.light_seen_at = -math.inf
        self.light_wait = False
        self.junction_until = -math.inf
        self.clear_since = None
        self.obstacle_wait = False

    def observe(self, observation, now, distance):
        # Called once per image, never once per control tick on a cached image.
        if observation.stop:
            self.stop_count += 1
            self.stop_seen_at = now
            if (self.stop_count >= 3 and observation.stop_distance < .65
                    and distance-self.stop_completed_at > .8):
                self.stop_pending = True
        else:
            self.stop_count = 0
        if observation.light != 'unknown':
            if observation.light == self.light_candidate:
                self.light_count += 1
            else:
                self.light_candidate, self.light_count = observation.light, 1
            if self.light_count >= 3:
                self.light_color = observation.light
                self.light_distance = observation.light_distance
                self.light_seen_at = now
        else:
            self.light_count = 0
        # Expire a completed STOP only after leaving it and losing its image.
        if distance-self.stop_completed_at > .8 and now-self.stop_seen_at > 1.5:
            self.stop_completed_at = -math.inf

    def step(self, now, distance, yaw, measured_speed, lane, front, sensors_fresh):
        s = self.settings
        if self.last_time is not None and now < self.last_time:
            self.__init__(s)
        self.last_time = now
        if not sensors_fresh or math.isnan(front):
            self.stop_stationary_since = None
            self.state = 'WAIT_SENSORS'
            return 0., 0.
        if front < s.stop_distance:
            self.obstacle_wait, self.clear_since = True, None
        elif self.obstacle_wait:
            if front < s.stop_distance+.08:
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = now
            elif now-self.clear_since >= .6:
                self.obstacle_wait = False
        if self.stop_pending:
            if abs(measured_speed) < .01:
                if self.stop_stationary_since is None:
                    self.stop_stationary_since = now
                elif now-self.stop_stationary_since >= s.stop_hold:
                    self.stop_pending = False
                    self.stop_completed_at = distance
                    self.stop_stationary_since = None
            else:
                self.stop_stationary_since = None
            self.state = 'STOP_HOLD'
            return 0., 0.
        if self.obstacle_wait:
            self.state = 'OBSTACLE_WAIT'
            return 0., 0.
        inside_junction = distance < self.junction_until
        light_recent = now-self.light_seen_at < 1.0
        if light_recent and self.light_distance < .65 and not inside_junction:
            if self.light_color in ('red', 'yellow'):
                self.light_wait = True
            elif self.light_color == 'green':
                self.light_wait = False
                if self.light_distance < .30:
                    self.junction_until = distance + .65
        if self.light_wait:
            self.state = 'LIGHT_WAIT'
            return 0., 0.
        if lane.confidence >= .35:
            self.last_lane, self.last_lane_time, self.last_yaw = lane, now, yaw
            self.state = 'FOLLOW'
        elif now-self.last_lane_time <= s.lane_grace:
            delta = math.atan2(math.sin(yaw-self.last_yaw), math.cos(yaw-self.last_yaw))
            lane = Lane(self.last_lane.lateral, self.last_lane.heading-delta, .2,
                        self.last_lane.target_y-.45*math.sin(delta))
            self.state = 'LANE_GRACE'
        else:
            self.state = 'LANE_LOST'
            return 0., 0.
        speed = s.max_speed * max(.3, min(1., lane.confidence))
        speed /= 1.+2.5*abs(lane.heading)
        if math.isfinite(front):
            speed *= max(0., min(1., (front-s.stop_distance)/.35))
        curvature = 2.*lane.target_y/(.45**2+lane.target_y**2)
        turn = max(-s.max_turn, min(s.max_turn, s.steering_gain*speed*curvature))
        return speed, turn
