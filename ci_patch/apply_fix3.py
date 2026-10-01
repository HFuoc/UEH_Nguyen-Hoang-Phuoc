from pathlib import Path

p = Path("src/crc_solution/crc_solution/control.py")
s = p.read_text()

s = s.replace(
    "from dataclasses import dataclass\nimport math\n",
    "from dataclasses import dataclass\nfrom collections import deque\nfrom statistics import median\nimport math\n",
    1,
)

needle = """        self.junction_turn_hint = 0.0
        self.junction_commit_until = -math.inf
"""
replacement = needle + """        # Distance-sampled curvature history.  A turn is preserved across a
        # STOP only if it was already sustained before the ambiguous stop-line
        # area; no map position or route-specific coordinate is stored.
        self.route_history = deque(maxlen=96)
"""
assert needle in s, "init turn-hint block not found"
s = s.replace(needle, replacement, 1)

needle = """            if (self.stop_armed and self.stop_count >= 3
                    and observation.stop_distance < .65):
                self.stop_pending = True
"""
replacement = """            if (self.stop_armed and self.stop_count >= 3
                    and observation.stop_distance < .65):
                if not self.stop_pending:
                    self.junction_turn_hint = self._historical_turn_intent(distance)
                self.stop_pending = True
"""
assert needle in s, "STOP pending block not found"
s = s.replace(needle, replacement, 1)

needle = """    def route_curvature_hint(self, distance):
        if distance < self.junction_commit_until and abs(self.junction_turn_hint) >= .60:
            return self.junction_turn_hint
        return 0.0

"""
replacement = """    def _remember_route(self, distance, curvature, lane):
        if (self.stop_pending or lane.confidence < .55 or curvature is None
                or not math.isfinite(curvature) or abs(curvature) > 2.2):
            return
        sample = (float(distance), float(curvature))
        # Distance sampling prevents slow motion from overweighting one place.
        if self.route_history and distance-self.route_history[-1][0] < .012:
            self.route_history[-1] = sample
        else:
            self.route_history.append(sample)

    def _historical_turn_intent(self, distance):
        # Exclude the nearest 0.25 m because STOP/cross-road paint is often
        # ambiguous there. The window is relative travelled odometry only.
        values = [(d, k) for d, k in self.route_history
                  if distance-.85 <= d <= distance-.25]
        if len(values) < 10 or values[-1][0]-values[0][0] < .28:
            return 0.0
        curvatures = [k for _, k in values]
        centre = float(median(curvatures))
        if abs(centre) < .72:
            return 0.0
        same = sum(1 for k in curvatures if k*centre > 0.)/len(curvatures)
        strong = sum(1 for k in curvatures
                     if k*centre > 0. and abs(k) >= .55)/len(curvatures)
        if same < .80 or strong < .65:
            return 0.0
        return max(-1.8, min(1.8, centre))

    def route_curvature_hint(self, distance):
        if distance < self.junction_commit_until and abs(self.junction_turn_hint) >= .60:
            return self.junction_turn_hint
        return 0.0

"""
assert needle in s, "route_curvature_hint block not found"
s = s.replace(needle, replacement, 1)

needle = """        if not sensors_fresh or math.isnan(front):
            self.stop_stationary_since = None
            self.state = 'WAIT_SENSORS'
            return 0., 0.
"""
replacement = needle + """        self._remember_route(distance, curvature, lane)
"""
assert needle in s, "sensor freshness block not found"
s = s.replace(needle, replacement, 1)

needle = """        if self.stop_pending:
            # At a marked intersection, lane paint is often ambiguous just
            # after the stop line.  Remember a strong pre-stop bend so a
            # transverse marking cannot reverse the route immediately after
            # the mandatory hold.
            if (abs(self.junction_turn_hint) < .60 and lane.confidence >= .55
                    and curvature is not None and math.isfinite(curvature)
                    and abs(curvature) >= .65):
                self.junction_turn_hint = max(-1.8, min(1.8, float(curvature)))
"""
replacement = """        if self.stop_pending:
            # Turn intent was frozen from a history window ending before the
            # ambiguous stop-line/cross-road area.
"""
assert needle in s, "instantaneous turn-hint block not found"
s = s.replace(needle, replacement, 1)

p.write_text(s)
print("fix3 historical STOP-turn intent applied")
