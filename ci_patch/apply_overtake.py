from pathlib import Path

root=Path(__file__).resolve().parents[1]
node=root/'src/crc_solution/crc_solution/node.py'
text=node.read_text()
text=text.replace(
    "from .sign_actions import SignActions\n",
    "from .sign_actions import SignActions\nfrom .overtake import OvertakeManager\n",
    1)
text=text.replace(
    "        self.sign_actions = SignActions()\n        self.classified_signs = []\n",
    "        self.sign_actions = SignActions()\n        self.overtake = OvertakeManager()\n        self.classified_signs = []\n",
    1)
text=text.replace(
    "                self.sign_actions = SignActions()\n                self.classified_signs = []\n",
    "                self.sign_actions = SignActions()\n                self.overtake = OvertakeManager()\n                self.classified_signs = []\n",
    1)
old="""        if fresh and self.crossing.update(self.people, self.scan_points, self.lane, now):
            speed, turn = 0., 0.
            self.controller.state = 'CROSSING_WAIT'
        if not self.freshness.fresh(now, time.monotonic(), self.settings.sensor_timeout):
"""
new="""        crossing_wait = fresh and self.crossing.update(self.people, self.scan_points, self.lane, now)
        if crossing_wait:
            speed, turn = 0., 0.
            self.controller.state = 'CROSSING_WAIT'
        # On a confirmed highway, intercept a persistent parked obstacle before
        # the conservative obstacle hold and execute a bounded relative-odometry
        # pass. STOP/light/crossing/sensor safety always retains priority.
        if (fresh and not crossing_wait and self.controller.state not in
                ('STOP_HOLD','LIGHT_WAIT','WAIT_SENSORS')):
            passing = self.overtake.command(self.sign_actions.highway,
                    self.scan_points, self.distance, self.yaw, now)
            if passing is not None:
                speed, turn, state = passing
                self.controller.state = state
        if not self.freshness.fresh(now, time.monotonic(), self.settings.sensor_timeout):
"""
if old not in text:
    raise SystemExit('node patch target not found')
node.write_text(text.replace(old,new,1))
