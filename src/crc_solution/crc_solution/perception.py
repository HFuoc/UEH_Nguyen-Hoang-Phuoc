"""Image-only lane and traffic-control observations.

The ground projection assumes the stock level camera. Camera height/offset are
robot calibration, not route coordinates. Ramp pitch remains a limitation.
"""
from dataclasses import dataclass, field
import math
import cv2
import numpy as np
from .curves import fit_curve


@dataclass
class Lane:
    lateral: float = 0.0
    heading: float = 0.0
    confidence: float = 0.0
    target_y: float = 0.0
    points: list = field(default_factory=list)
    curvature: float = None


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
        self.curve_active = False
        self.last_curve = 0.

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
        # White paint is nearly neutral. Blue-grey light leaks on tunnel wall
        # posts otherwise form convincing false lane pairs above the road.
        mask = ((gray > threshold) & (hsv[:, :, 1] < 30) &
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
                    # In right-hand travel the continuous outer boundary is
                    # on the right; the centre marking may be dashed/absent.
                    # Do not reinterpret that same edge as the left boundary
                    # when the robot is displaced outside it. An inferred
                    # centre outside the image is still a useful correction.
                    for center, bias in ((peak-width/2,-.2), (peak+width/2,.5)):
                        if abs(center-prediction) < .9*width:
                            candidates.append((abs(center-prediction)/width+bias, center, .45))
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
                slope = (points_arr[j,1]-points_arr[i,1])/(points_arr[j,0]-points_arr[i,0])
                intercept = points_arr[i,1]-slope*points_arr[i,0]
                keep = np.abs(slope*points_arr[:,0]+intercept-points_arr[:,1]) < .025
                score = float(weights_arr[keep].sum())
                if best is None or score > best[0]: best = (score,keep)
        if best is not None and np.count_nonzero(best[1]) >= 3:
            points_arr, weights_arr = points_arr[best[1]], weights_arr[best[1]]
        fit = np.polyfit(points_arr[:, 0], points_arr[:, 1], 1, w=weights_arr)
        residual = np.sqrt(np.average((np.polyval(fit, points_arr[:, 0])-points_arr[:, 1])**2, weights=weights_arr))
        confidence = min(1., sum(weights_arr)/3.) * max(0., 1.-residual/.06)
        near = float(np.polyval(fit, .30))
        self.last_center = float(np.clip(.5*self.last_center+.5*near, -.07, .07))
        lane = Lane(near, math.atan(float(fit[0])), confidence,
                    float(np.polyval(fit, .45)), overlay)
        # A single straight regression can connect a dashed inner edge to
        # the wrong outer boundary on a tight bend. Require paired spatial
        # support for a curved corridor before overriding that regression.
        k, offset, score, support = fit_curve(mask, self, self.last_curve)
        threshold = .43 if self.curve_active else .50
        paired = .10 if self.curve_active else .20
        active = (abs(k) > 1. and score > threshold and min(support) > paired)
        self.curve_active = active
        if active:
            self.last_curve = k
            theta = .70*k
            x = math.sin(theta)/k
            y = offset+(1.-math.cos(theta))/k
            curvature = 2*y/(x*x+y*y)
            lane = Lane(offset, math.atan(.45*k), min(.85, score),
                        y, overlay, curvature)
        return lane, mask

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
                perimeter = cv2.arcLength(contour, True)
                circularity = 4*math.pi*area/max(perimeter**2, 1)
                # Require a bounded, tall housing around the lamp. A dark
                # background alone is not housing: otherwise an octagonal
                # STOP in shadow becomes red and can never release the car.
                # Relative contrast also preserves a black housing against a
                # dim background rather than using a fixed darkness test.
                left, right = max(0, x-2*bw), min(w, x+3*bw)
                top, bottom = max(0, y-4*bh), min(h, y+5*bh)
                context = val[top:bottom, left:right]
                dark_limit = min(65., float(np.percentile(context, 75))-5.)
                housing = False
                # A clipped STOP and its black backing can look like a round
                # lamp in a tall housing. Require the whole coloured component
                # and housing to be visible before using their size as range.
                complete = (x > 1 and x+bw < w-1 and y > 1
                            and y+bh < int(min(.65*h, self.cy+15))-1)
                if complete and dark_limit > 0 and circularity > .50 and bw < .12*w:
                    dark = (context < dark_limit).astype(np.uint8)
                    surrounds, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL,
                                                    cv2.CHAIN_APPROX_SIMPLE)
                    center_x, center_y = x+bw/2-left, y+bh/2-top
                    for surround in surrounds:
                        sx, sy, sw, sh = cv2.boundingRect(surround)
                        if (sx <= center_x <= sx+sw and sy <= center_y <= sy+sh
                                and left+sx > 0 and left+sx+sw < w
                                and top+sy > 0 and top+sy+sh < h
                                and 1.1*bw <= sw <= 4*bw and sh >= 2.2*bh
                                and .12 < sw/max(sh, 1) < .85
                                and cv2.contourArea(surround)/(sw*sh) > .35):
                            housing = True
                            break
                lamp = housing
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
