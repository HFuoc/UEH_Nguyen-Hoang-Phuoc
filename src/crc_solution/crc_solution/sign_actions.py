"""Confirm independent sign observations and apply bounded caution zones."""
import math


class SignActions:
    def __init__(self):
        self.counts={}
        self.seen={}
        self.handled={}
        self.caution_until=-math.inf
        self.highway=False
        self.label='unknown'
        self.last_time=-math.inf

    def update(self, detections, now, distance):
        if now < self.last_time:
            self.__init__()
        self.last_time=now
        visible={item.label for item in detections if 0. < item.distance < 1.2}
        self.label=next((d.label for d in detections if d.label in visible),'unknown')
        for label in set(self.counts) | visible:
            previous=self.seen.get(label,-math.inf)
            # Rearm on the observed gap, before the next confirmation frames
            # overwrite its timestamp. Previously the second frame checked
            # the immediately preceding frame, so rearming was impossible.
            if label in self.handled:
                old_distance, _=self.handled[label]
                if distance-old_distance >= 1.0 and now-previous >= 1.0:
                    del self.handled[label]
                    self.counts[label]=0
            if label not in visible:
                self.counts[label]=0
                continue
            self.counts[label]=self.counts.get(label,0)+1 if now-previous < .6 else 1
            self.seen[label]=now
            if self.counts[label] < 2:
                continue
            if label in self.handled:
                continue
            self.handled[label]=(distance,now)
            if label in ('ramp','tunnel','uneven','crosswalk','bus'):
                # Odometric extent measured from the observed sign, never a
                # location on the supplied map. Further signs can extend it.
                self.caution_until=max(self.caution_until,distance+2.5)
            elif label=='hw_entry':
                self.highway=True
            elif label=='hw_exit':
                self.highway=False

    def speed_cap(self, distance):
        return .10 if distance < self.caution_until else math.inf
