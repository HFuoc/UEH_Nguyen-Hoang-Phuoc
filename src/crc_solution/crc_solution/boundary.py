"""Trace a continuous right lane edge in camera ground projection."""
from dataclasses import dataclass
import cv2
import numpy as np


def runs(values):
    indices=np.flatnonzero(values)
    if not len(indices): return []
    groups=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1)
    return [(float(g.mean()),len(g)) for g in groups]


@dataclass
class Trace:
    valid: bool = False
    curvature: float = 0.
    target_x: float = 0.
    target_y: float = 0.
    confidence: float = 0.
    boundary: object = None
    center: object = None
    reason: str = ''


def trace_lane(mask, camera, prior_center=0.):
    """Follow a thin, continuous boundary, then offset along its left normal."""
    resolution = .004
    forward = np.arange(.15, 1.001, resolution)
    lateral = np.arange(-.8, .801, resolution)
    x, y = np.meshgrid(forward, lateral, indexing='ij')
    u = (camera.cx-y*camera.fx/(x-camera.offset)).astype('float32')
    v = (camera.cy+camera.height*camera.fy/(x-camera.offset)).astype('float32')
    bird = cv2.remap(mask, u, v, cv2.INTER_NEAREST)
    if np.mean(bird > 0) > .30:
        return Trace(reason='broad bright surface')
    rows = []
    for i in range(1, len(forward)-1, 3):
        if forward[i] < .245:
            continue
        white = np.mean(bird[i-1:i+2], axis=0) > 127
        candidates = []
        for middle, width in runs(white):
            if .004 <= width*resolution <= .085:
                col = int(round(middle))
                if 3 < u[i, col] < mask.shape[1]-3 and camera.cy+15 < v[i, col] < mask.shape[0]-3:
                    candidates.append((float(lateral[col]), width*resolution))
        rows.append((float(forward[i]), candidates))
    paths = []
    for seed_index, (seed_x, candidates) in enumerate(rows):
        if seed_x > .33:
            break
        for seed_y, seed_width in candidates:
            if abs(seed_y-(prior_center-.175)) > .33:
                continue
            path = [(seed_x, seed_y)]
            errors = []
            slope = 0.
            for px, candidates in rows[seed_index+1:]:
                gap = px-path[-1][0]
                if gap > .13:
                    break
                if len(path) >= 3:
                    recent = np.asarray(path[-7:])
                    slope = float(np.polyfit(recent[:, 0], recent[:, 1], 1)[0])
                predicted = path[-1][1]+slope*gap
                options = [(abs(py-predicted), py) for py, width in candidates
                           if abs(py-predicted) < .026+.10*gap]
                if not options:
                    continue
                error, py = min(options)
                path.append((px, py))
                errors.append(error)
            span = path[-1][0]-path[0][0]
            if len(path) < 7 or span < .18:
                continue
            # Favor a nearby right edge and continuous evidence. The prior
            # describes local lane offset, never a stored track coordinate.
            score = min(span, .60)-.35*abs(seed_y-(prior_center-.175))
            score -= .5*(seed_x-.25)+2.*float(np.mean(errors))
            paths.append((score, path))
    if not paths:
        return Trace(reason='insufficient continuous edge')
    score, path = max(paths, key=lambda item: item[0])
    boundary = np.asarray(path)
    # Fit only the local steering horizon. Distant lane merges must not pull
    # the near corridor sideways through a long polynomial regression.
    local = boundary[:, 0] <= .70
    if np.count_nonzero(local) >= 7:
        boundary = boundary[local]
    # A local quadratic smooths raster stair steps without extrapolating the
    # chosen lookahead beyond the measured boundary.
    fit = np.polyfit(boundary[:, 0], boundary[:, 1], 2)
    py = np.polyval(fit, boundary[:, 0])
    derivative = np.polyval(np.polyder(fit), boundary[:, 0])
    norm = np.sqrt(1.+derivative**2)
    center = np.column_stack((boundary[:, 0]-.175*derivative/norm,
                              py+.175/norm))
    useful = center[:, 0] > .12
    center = center[useful]
    if len(center) < 4:
        return Trace(reason='center outside forward view', boundary=boundary.tolist())
    distance = np.linalg.norm(center, axis=1)
    target = center[np.argmin(abs(distance-.50))]
    residual = float(np.sqrt(np.mean((py-boundary[:, 1])**2)))
    span = float(boundary[-1, 0]-boundary[0, 0])
    confidence = min(.85, span/.5)*max(0., 1.-residual/.05)
    return Trace(confidence >= .30, float(2*target[1]/np.dot(target, target)),
                 float(target[0]), float(target[1]), confidence,
                 boundary.tolist(), center.tolist())

