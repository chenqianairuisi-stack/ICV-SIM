"""Route sampling and collision-filtered Reeds-Shepp parking."""
import math
from pathlib import Path
import sys
import numpy as np
from shapely.geometry import box
from .model import wrap


class Route:
    def __init__(self, points, direction=1):
        self.points = np.asarray(points, dtype=float)
        if len(self.points) < 2:
            raise ValueError("A route needs at least two distinct points")
        ds = np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1)
        keep = np.r_[True, ds > 1e-6]
        self.points = self.points[keep]
        self.s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1))]
        self.length = float(self.s[-1])
        self.direction = direction
        tangent = np.arctan2(np.gradient(self.points[:, 1]), np.gradient(self.points[:, 0]))
        self.yaw = np.unwrap(tangent)
        self.curvature = np.gradient(self.yaw, self.s)

    def at(self, s):
        s = np.clip(s, 0, self.length)
        return np.array([np.interp(s, self.s, self.points[:, 0]),
                         np.interp(s, self.s, self.points[:, 1])])

    def project(self, x, y, hint=0.0):
        # Bound the search to avoid jumping between adjacent legs of a route.
        lo = max(0, np.searchsorted(self.s, hint - 5) - 1)
        hi = min(len(self.s) - 1, np.searchsorted(self.s, hint + 45) + 1)
        a, b = self.points[lo:hi, :2], self.points[lo + 1:hi + 1, :2]
        ab = b - a
        t = np.clip(np.sum((np.array([x, y]) - a) * ab, axis=1) /
                    np.maximum(np.sum(ab * ab, axis=1), 1e-12), 0, 1)
        q = a + ab * t[:, None]
        i = int(np.argmin(np.sum((q - [x, y]) ** 2, axis=1)))
        idx = lo + i
        s = self.s[idx] + t[i] * (self.s[idx + 1] - self.s[idx])
        vec = ab[i]
        cross = (vec[0] * (y - q[i, 1]) - vec[1] * (x - q[i, 0])) / np.linalg.norm(vec)
        return float(s), float(cross), float(math.atan2(vec[1], vec[0]))


def straight(length, y=1.75, start=0):
    x = np.linspace(start, start + length, int(length / .15) + 2)
    return Route(np.c_[x, np.full_like(x, y)])


def lane_change(length=65, start=1.5, end=7.0):
    x = np.arange(0, length + .1, .1)
    u = np.clip((x - start) / (end - start), 0, 1)
    y = 1.75 + 3.5 * (10 * u**3 - 15 * u**4 + 6 * u**5)
    return Route(np.c_[x, y])


def parking_plan(vehicle, area, obstacles, start=(0, 3.5, 0), goal=(22, -4.5, math.pi / 2)):
    vendor = str(Path(__file__).resolve().parents[1] / "vendor")
    if vendor not in sys.path:
        sys.path.insert(0, vendor)
    import reeds_shepp_path_planning as rs
    rs.show_animation = False
    staging = (28., start[1], start[2])
    candidates = rs.calc_paths(*staging, *goal,
                               math.tan(vehicle.max_steer * .88) / vehicle.wheelbase, .08)
    valid = []
    for path in candidates:
        safe = True
        for x, y, yaw in zip(path.x, path.y, path.yaw):
            poly = vehicle.polygon(x, y, yaw, .08)
            if not area.covers(poly) or any(poly.intersects(o) for o in obstacles):
                safe = False
                break
        if safe and path.directions[-1] == -1:
            shifts = sum(a != b for a, b in zip(path.directions, path.directions[1:]))
            valid.append((path.L + shifts * 4, path))
    if not valid:
        raise RuntimeError("No collision-free reverse parking route for this geometry")
    path = min(valid, key=lambda p: p[0])[1]
    segments, begin = [straight(staging[0] - start[0], start[1], start[0])], 0
    for i in range(1, len(path.x)):
        if path.directions[i] != path.directions[i - 1]:
            pts = np.c_[path.x[begin:i], path.y[begin:i]]
            if len(pts) > 1:
                segments.append(Route(pts, path.directions[i - 1]))
            begin = i - 1
    segments.append(Route(np.c_[path.x[begin:], path.y[begin:]], path.directions[-1]))
    # Exit along the reverse entry route: collision checks use the same footprint.
    exit_route = Route(segments[-1].points[::-1], 1)
    end = exit_route.points[-1]
    exit_heading = math.atan2(end[1] - exit_route.points[-2, 1], end[0] - exit_route.points[-2, 0])
    extension = end + np.arange(.1, 13, .1)[:, None] * [math.cos(exit_heading), math.sin(exit_heading)]
    exit_route = Route(np.vstack([exit_route.points, extension]), 1)
    return segments, exit_route, goal
