"""Local collision geometry. Uses robot calibration and onboard scans only."""
import math
import numpy as np


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    values = np.asarray(ranges, dtype=float)
    if (not values.size or not all(math.isfinite(v) for v in
            (angle_min, angle_increment, range_min, range_max))
            or angle_increment == 0 or range_min < 0 or range_max <= range_min):
        return None
    angles = angle_min + np.arange(values.size)*angle_increment
    valid = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    sector = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) < 1.2
    if not np.any(sector) or np.mean((valid | np.isposinf(values))[sector]) < .5:
        return None
    # Remove isolated centimetre-scale noise on continuous surfaces, without
    # filling gaps or removing narrow objects at depth discontinuities.
    neighbors=np.stack([np.roll(values,shift) for shift in (-1,0,1)])
    neighbor_valid=np.stack([np.roll(valid,shift) for shift in (-1,0,1)])
    continuous=np.all(neighbor_valid,axis=0)
    continuous &= np.ptp(np.where(neighbor_valid,neighbors,0.),axis=0) < .06
    continuous[[0,-1]]=False
    values=values.copy()
    values[continuous]=np.median(neighbors[:,continuous],axis=0)
    x = values[valid]*np.cos(angles[valid])-.064
    y = values[valid]*np.sin(angles[valid])
    # Stock Waffle self-return envelope. At a ramp transition, 1 cm range
    # noise spreads body returns into the front wheel/body corners. The
    # front mask includes the maximum wheel width behind the nose; it does
    # not remove the rear-side points needed to check tail swing on bends.
    own_body = (((x > -.21) & (x < .10) & (np.abs(y) < .14))
                | ((x > -.065) & (x < .10) & (np.abs(y) < .163)))
    return np.column_stack((x[~own_body], y[~own_body]))


def enclosed_corridor(points):
    """Detect extended walls on both sides, not an isolated pole or actor."""
    if points is None or len(points)<20:
        return False
    nearby=points[(points[:,0]>-.25)&(points[:,0]<.8)]
    left=nearby[(nearby[:,1]>.15)&(nearby[:,1]<.9)]
    right=nearby[(nearby[:,1]<-.10)&(nearby[:,1]>-.65)]
    return (len(left)>=10 and len(right)>=10
            and np.ptp(left[:,0])>.3 and np.ptp(right[:,0])>.3)


def steering_curvature(lane, gain=1., lookahead=.45):
    # Shorten only in a measured enclosed corridor. The distant heading is
    # needed on open-road tapers to avoid following a nearby crossing stripe.
    if lane.curvature is not None and lookahead >= .4:
        return float(np.clip(gain*lane.curvature, -5., 5.))
    target = lane.lateral if lookahead < .4 else lane.target_y
    return float(np.clip(gain*2.*target/(lookahead**2+target**2), -5., 5.))


def swept_clearance(points, curvature, horizon=.8):
    """Distance to collision along the curved stock body and wheel footprint.

    Return travel before contact plus the 0.10 m nose offset, retaining the
    straight-corridor stop_distance convention. Side/rear sweep is included.
    A stop threshold of .30 therefore leaves about .20 m of travel margin.
    """
    if points is None or not math.isfinite(curvature):
        return math.nan
    if not len(points):
        return math.inf
    points = points[np.hypot(points[:, 0], points[:, 1]) < horizon+.3]
    if not len(points):
        return math.inf
    travel = np.arange(0., horizon+.01, .01)
    theta = travel*curvature
    if abs(curvature) < 1e-5:
        px, py = travel, np.zeros_like(travel)
    else:
        px, py = np.sin(theta)/curvature, (1.-np.cos(theta))/curvature
    dx, dy = points[:, 0, None]-px, points[:, 1, None]-py
    along = dx*np.cos(theta)+dy*np.sin(theta)
    across = -dx*np.sin(theta)+dy*np.cos(theta)
    # The rear body is narrower than the wheels. A box of maximum width over
    # the entire length incorrectly reports rear-corner contact on tight turns.
    body = (along >= -.202) & (along <= .09) & (np.abs(across) <= .138)
    wheels = (np.abs(along) <= .04) & (np.abs(across) <= .158)
    hit = np.any(body | wheels, axis=0)
    return float(travel[np.flatnonzero(hit)[0]]+.10) if np.any(hit) else math.inf


def choose_curvature(points, desired, previous=0.):
    """Small local steering corrections; never plan a lane change around a car.

    Prefer the camera path when clear. Otherwise search nearby curvatures for
    a body-safe path, accounting for rear-corner swing before a narrow bend.
    The limited correction is for lane estimation error, not overtaking.
    """
    if points is None:
        return desired, math.nan
    desired = float(np.clip(desired, -2.2, 2.2))
    base = swept_clearance(points, desired)
    if base >= .65:
        return desired, base
    candidates = np.unique(np.r_[desired, np.arange(-2.2, 2.21, .1)])
    candidates = candidates[np.abs(candidates-desired) <= 1.4]
    options=[]
    for curvature in candidates:
        clearance = swept_clearance(points, curvature)
        score = (4.*min(clearance, .65)-.16*abs(curvature-desired)
                 -.04*abs(curvature-previous))
        options.append((score, float(curvature), clearance))
    _, curvature, clearance = max(options)
    return curvature, clearance
