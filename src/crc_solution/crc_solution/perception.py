"""Image-only lane and traffic-control observations.

The ground projection assumes the stock level camera. Camera height/offset are
robot calibration, not route coordinates. Ramp pitch remains a limitation.
"""
from dataclasses import dataclass, field
import math
import cv2
import numpy as np


@dataclass
class Lane:
    lateral: float = 0.0
    heading: float = 0.0
    confidence: float = 0.0
    target_y: float = 0.0
    points: list = field(default_factory=list)


@dataclass
class Signs:
    stop: bool = False
    stop_distance: float = math.inf
    light: str = 'unknown'
    light_distance: float = math.inf
    crosswalk: bool = False
    boxes: list = field(default_factory=list)


def runs(values):
    indices = np.flatnonzero(values)
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [(float(g.mean()), len(g)) for g in groups]


class Perception:
    def __init__(self):
        self.fx = self.fy = 381.36
        self.cx, self.cy = 320.0, 240.0
        self.height = 0.117
        self.offset = 0.069
        self.last_center = 0.0

    def calibrate(self, matrix):
        if matrix[0] > 0 and matrix[4] > 0:
            self.fx, self.fy = float(matrix[0]), float(matrix[4])
            self.cx, self.cy = float(matrix[2]), float(matrix[5])

    def lane(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # A local contrast requirement keeps uniform dim asphalt out of the mask.
        smooth = cv2.GaussianBlur(gray, (31, 31), 0)
        contrast = gray.astype(np.int16) - smooth.astype(np.int16)
        threshold = max(32.0, float(np.percentile(gray[image.shape[0]//2:], 90)) * .65)
        mask = ((gray > threshold) & (hsv[:, :, 1] < 90) &
                ((contrast > 9) | (gray > 180))).astype(np.uint8) * 255
        h, w = gray.shape
        points, weights, overlay = [], [], []
        # The ramp is a broad bright surface that occludes the painted road.
        # Its visible side can stand in for a lane boundary; require support
        # across several rows so one zebra stripe is not treated as a ramp.
        broad_rows = 0
        for fraction in [.91,.83,.75,.67]:
            row=int(h*fraction)
            if any(size > .45*w for _,size in runs(np.mean(mask[row-2:row+3],axis=0)>120)):
                broad_rows += 1
        for fraction in [.95, .91, .87, .83, .79, .75, .71, .67, .63, .60, .58]:
            row = int(h * fraction)
            if row <= self.cy + 12:
                continue
            depth = self.height * self.fy / (row - self.cy)
            width = self.fx * .35 / depth
            if not 25 < width < 1.5*w:
                continue
            row_runs = runs(np.mean(mask[row-2:row+3], axis=0) > 120)
            peaks = [x for x, size in row_runs
                     if 1 <= size <= max(18, width*.13)]
            prediction = self.cx - self.last_center * self.fx / depth
            candidates = []
            for left in peaks:
                for right in peaks:
                    separation = right-left
                    if .65*width < separation < 1.4*width:
                        center = (left+right)/2
                        score = abs(center-prediction)/width + .8*abs(separation/width-1)
                        candidates.append((score, center, 1.0))
            if not candidates:
                for peak in peaks:
                    for center in (peak-width/2, peak+width/2):
                        if 0 <= center < w and abs(center-prediction) < .4*width:
                            candidates.append((abs(center-prediction)/width+.3, center, .45))
            if broad_rows >= 3:
                for middle,size in row_runs:
                    if size < .45*w:
                        continue
                    right = middle+(size-1)/2
                    center = right-width/2
                    if middle-size/2 < self.cx < right and right < w-3 and 0 <= center < w:
                        candidates.append((abs(center-prediction)/width+.20,center,.55))
            if not candidates:
                continue
            score, center, weight = min(candidates)
            if score > 1.10:
                continue
            lateral = -(center-self.cx)*depth/self.fx
            points.append((depth+self.offset, lateral))
            weights.append(weight)
            overlay.append((int(center), row))
        if len(points) < 2:
            self.last_center *= .5
            return Lane(points=overlay), mask
        points_arr = np.asarray(points)
        weights_arr = np.asarray(weights)
        # Distant intersections and tapering lane edges must not overwhelm the
        # lane underneath the robot. Fit the visible near corridor first.
        near_rows = points_arr[:, 0] <= .52
        if np.count_nonzero(near_rows) >= 3:
            points_arr, weights_arr = points_arr[near_rows], weights_arr[near_rows]
        # Remove isolated stripe/crossbar candidates with a deterministic small
        # consensus fit. Keep the majority rather than fitting one remote mark.
        best = None
        for i in range(len(points_arr)):
            for j in range(i+1, len(points_arr)):
                if abs(points_arr[j,0]-points_arr[i,0]) < .05:
                    continue
                model = np.polyfit(points_arr[[i,j],0], points_arr[[i,j],1], 1)
                keep = np.abs(np.polyval(model,points_arr[:,0])-points_arr[:,1]) < .025
                score = float(weights_arr[keep].sum())
                if best is None or score > best[0]: best = (score,keep)
        if best is not None and np.count_nonzero(best[1]) >= 3:
            points_arr, weights_arr = points_arr[best[1]], weights_arr[best[1]]
        fit = np.polyfit(points_arr[:, 0], points_arr[:, 1], 1, w=weights_arr)
        residual = np.sqrt(np.average((np.polyval(fit, points_arr[:, 0])-points_arr[:, 1])**2, weights=weights_arr))
        confidence = min(1., sum(weights_arr)/3.) * max(0., 1.-residual/.06)
        near = float(np.polyval(fit, .30))
        self.last_center = float(np.clip(.5*self.last_center+.5*near, -.07, .07))
        return Lane(near, math.atan(float(fit[0])), confidence,
                    float(np.polyval(fit, .45)), overlay), mask

    def signs(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        colors = {
            'red': (((hue < 12) | (hue > 170)) & (((sat > 120) & (val > 80)) | ((sat > 35) & (val > 170)))),
            'yellow': ((hue > 17) & (hue < 38) & (((sat > 120) & (val > 100)) | ((sat > 35) & (val > 170)))),
            'green': ((hue > 38) & (hue < 90) & (((sat > 100) & (val > 65)) | ((sat > 25) & (val > 170))))}
        result = Signs()
        h, w = val.shape
        for color, pixels in colors.items():
            pixels[int(min(.65*h, self.cy+15)):] = False
            contours, _ = cv2.findContours(pixels.astype(np.uint8), cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 6:
                    continue
                x, y, bw, bh = cv2.boundingRect(contour)
                if not .45 < bw/max(bh, 1) < 1.7 or bw > .35*w:
                    continue
                # A lamp sits inside a tall dark housing; inspect its adjacent
                # vertical column. A coloured plate does not have this housing.
                left, right = max(0, x-bw//2), min(w, x+bw+bw//2+1)
                above = val[max(0,y-3*bh):y, left:right]
                below = val[y+bh:min(h,y+4*bh), left:right]
                darkness = max(float(np.mean(a < 65)) if a.size else 0. for a in (above, below))
                perimeter = cv2.arcLength(contour, True)
                circularity = 4*math.pi*area/max(perimeter**2, 1)
                lamp = darkness > .62 and circularity > .50 and bw < .12*w
                if lamp:
                    distance = self.fx*.034/max(bw, 1)
                    if distance < result.light_distance:
                        result.light, result.light_distance = color, distance
                    result.boxes.append((x,y,bw,bh,color))
                elif color == 'red' and area > 25 and bw > 9:
                    polygon = cv2.approxPolyDP(contour, .035*perimeter, True)
                    # Warning triangles must not be treated as STOP signs.
                    if 6 <= len(polygon) <= 10 and .7 < bw/bh < 1.3 and area/(bw*bh) > .62:
                        result.stop = True
                        result.stop_distance = min(result.stop_distance, self.fx*.07/bw)
                        result.boxes.append((x,y,bw,bh,'STOP'))
        # Wide horizontal stripe groups warn of a crossing; they do not imply
        # that the road is occupied. LiDAR makes the occupancy decision.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        rows = np.mean(gray[int(.57*h):int(.88*h), int(.25*w):int(.75*w)] > 170, axis=1)
        result.crosswalk = len(runs(rows > .55)) >= 3
        return result


def corridor_range(ranges, angle_min, angle_increment, range_min, range_max,
                   half_width=.17, lidar_offset=-.064):
    """Nearest return in the swept body corridor, using LaserScan geometry."""
    values = np.asarray(ranges, dtype=float)
    if values.size == 0 or not math.isfinite(angle_increment) or angle_increment == 0:
        return math.nan
    angles = angle_min + np.arange(values.size)*angle_increment
    valid = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    front_sector = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) < .55
    # Positive infinity means clear; NaN/negative/zero is not a clear reading.
    usable = valid | np.isposinf(values)
    if not np.any(front_sector) or np.mean(usable[front_sector]) < .5:
        return math.nan
    x = np.full(values.shape, math.inf)
    y = np.full(values.shape, math.inf)
    x[valid] = values[valid]*np.cos(angles[valid])+lidar_offset
    y[valid] = values[valid]*np.sin(angles[valid])
    # Rays intersecting the robot's own footprint are common during pitching
    # on the ramp. The stock body ends behind x=0.10 m in base_footprint.
    # Check the corridor ahead of the nose, not lateral points beside wheels.
    inside = valid & (x > .10) & (np.abs(y) < half_width)
    return float(np.min(x[inside])) if np.any(inside) else math.inf
