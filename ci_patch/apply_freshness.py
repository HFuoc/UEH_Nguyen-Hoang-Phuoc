from pathlib import Path

def replace_once(path, old, new):
    p = Path(path)
    s = p.read_text()
    if old not in s:
        raise SystemExit(f"pattern not found in {path}: {old[:80]!r}")
    if s.count(old) != 1:
        raise SystemExit(f"pattern not unique in {path}: {s.count(old)}")
    p.write_text(s.replace(old, new, 1))

replace_once(
    "src/crc_solution/crc_solution/freshness.py",
    """    def fresh(self, now, wall, timeout):
        return all(wall-self.received.get(k, -math.inf) < timeout
                   and -.2 <= now-self.stamps.get(k, -math.inf) < timeout
                   for k in ('image', 'scan', 'odom'))
""",
    """    def fresh_names(self, now, wall, timeout, names):
        return all(wall-self.received.get(k, -math.inf) < timeout
                   and -.2 <= now-self.stamps.get(k, -math.inf) < timeout
                   for k in names)

    def fresh(self, now, wall, timeout):
        return self.fresh_names(now, wall, timeout, ('image', 'scan', 'odom'))

    def age(self, name, now, wall):
        return (wall-self.received.get(name, -math.inf),
                now-self.stamps.get(name, -math.inf))
"""
)

replace_once(
    "src/crc_solution/crc_solution/control.py",
    "    sensor_timeout: float = .8\n",
    "    sensor_timeout: float = .8\n    camera_timeout: float = 1.6\n"
)
replace_once(
    "src/crc_solution/crc_solution/control.py",
    "                    'stop_distance':(.20, .8), 'sensor_timeout':(.1, 3.),\n",
    "                    'stop_distance':(.20, .8), 'sensor_timeout':(.1, 3.),\n                    'camera_timeout':(.3, 3.),\n"
)

replace_once(
    "src/crc_solution/crc_solution/node.py",
    """        fresh = (self.freshness.fresh(now, wall, self.settings.sensor_timeout)
                 and wall-self.clock_changed_wall < self.settings.sensor_timeout)
        if self.image_sequence != self.processed_sequence and fresh and wall-self.last_process_wall >= .1:
""",
    """        # Scan/odometry and /clock are motion-critical and keep the strict
        # watchdog. Gazebo's rendered camera may lag under software rendering,
        # so only the camera receives a larger hard timeout. Frames older than
        # the normal timeout are never treated as a fresh lane measurement.
        motion_fresh = (self.freshness.fresh_names(
            now, wall, self.settings.sensor_timeout, ('scan', 'odom'))
            and wall-self.clock_changed_wall < self.settings.sensor_timeout)
        camera_hard_fresh = self.freshness.fresh_names(
            now, wall, self.settings.camera_timeout, ('image',))
        camera_nominal_fresh = self.freshness.fresh_names(
            now, wall, self.settings.sensor_timeout, ('image',))
        fresh = motion_fresh and camera_hard_fresh
        if self.image_sequence != self.processed_sequence and fresh and wall-self.last_process_wall >= .1:
"""
)

replace_once(
    "src/crc_solution/crc_solution/node.py",
    """        desired = steering_curvature(self.lane, self.settings.steering_gain, lookahead)
        if self.lane.confidence < MIN_LANE_CONFIDENCE:
            desired = self.controller.last_path_curvature
        self.curvature, self.front = choose_curvature(self.scan_points,
            desired, self.curvature)
        speed, turn = self.controller.step(now, self.distance, self.yaw, self.speed,
                                            self.lane, self.front, fresh, self.curvature)
""",
    """        lane_for_control = self.lane if camera_nominal_fresh else Lane()
        desired = steering_curvature(lane_for_control, self.settings.steering_gain, lookahead)
        if lane_for_control.confidence < MIN_LANE_CONFIDENCE:
            desired = self.controller.last_path_curvature
        self.curvature, self.front = choose_curvature(self.scan_points,
            desired, self.curvature)
        speed, turn = self.controller.step(now, self.distance, self.yaw, self.speed,
                                            lane_for_control, self.front, fresh, self.curvature)
"""
)

replace_once(
    "src/crc_solution/crc_solution/node.py",
    """        if not self.freshness.fresh(now, time.monotonic(), self.settings.sensor_timeout):
            speed, turn = 0., 0.
            self.controller.state = 'WAIT_SENSORS'
""",
    """        # Re-check immediately before publishing: camera may use its
        # bounded hard grace, but scan/odom and /clock stay strict.
        publish_wall = time.monotonic()
        publish_fresh = (self.freshness.fresh_names(
            now, publish_wall, self.settings.sensor_timeout, ('scan', 'odom'))
            and self.freshness.fresh_names(
                now, publish_wall, self.settings.camera_timeout, ('image',))
            and publish_wall-self.clock_changed_wall < self.settings.sensor_timeout)
        if not publish_fresh:
            speed, turn = 0., 0.
            self.controller.state = 'WAIT_SENSORS'
"""
)
