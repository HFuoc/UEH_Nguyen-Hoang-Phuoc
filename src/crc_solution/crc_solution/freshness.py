"""Receipt and source-time watchdogs, independent of ROS for testing."""
import math


class SensorFreshness:
    def __init__(self):
        self.received = {}
        self.stamps = {}

    def accept(self, name, stamp, wall, now=None):
        if not math.isfinite(stamp) or stamp < 0 or stamp <= self.stamps.get(name, -math.inf):
            return False
        # Reject before storing: a future outlier must not prevent subsequent
        # correctly stamped samples from passing the monotonicity check.
        if now is not None and (not math.isfinite(now) or stamp > now+.2):
            return False
        self.stamps[name], self.received[name] = stamp, wall
        return True

    def invalidate(self, name):
        self.received.pop(name, None)

    def clear(self):
        self.received.clear()
        self.stamps.clear()

    def fresh(self, now, wall, timeout):
        return all(wall-self.received.get(k, -math.inf) < timeout
                   and -.2 <= now-self.stamps.get(k, -math.inf) < timeout
                   for k in ('image', 'scan', 'odom'))
