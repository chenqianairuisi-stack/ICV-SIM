"""Explicit synthetic geometries, never represented as official NEVC maps."""
from dataclasses import dataclass, field
import math
import numpy as np
from scipy.interpolate import CubicSpline
from shapely.geometry import LineString, box
from shapely.ops import unary_union
from .planning import Route, straight, lane_change, parking_plan


@dataclass
class Scenario:
    key: str
    title: str
    routes: list
    road: object
    initial: tuple
    speed: float
    timeout: float = 300
    obstacles: list = field(default_factory=list)
    lights: list = field(default_factory=list)
    limits: list = field(default_factory=list)
    slot: object = None
    parking_goal: tuple = None
    dwell_after: int = -1
    dwell: float = 10.5
    base: int = 100
    official_example: bool = False
    metadata: dict = field(default_factory=dict)

    def objects(self, t):
        return [[o[0] + o[5] * t * math.cos(o[2]),
                 o[1] + o[5] * t * math.sin(o[2]), *o[2:]] for o in self.obstacles]


def light_state(light, t):
    if light.get("fixed"):
        return light["fixed"]
    phase = (t + light.get("offset", 0)) % light.get("period", 40)
    return "red" if phase < light.get("red", 20) else "green"


def make_city():
    x = np.linspace(0, 3500, 16001)
    y = 38 * np.sin(x / 165) + 16 * np.sin(x / 61)
    p = np.c_[x, y]
    length = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
    p *= 3680 / length
    base = Route(p)
    s = np.arange(0, base.length, .4)
    s = np.r_[s, base.length]
    points = np.array([base.at(t) for t in s])
    tangent = np.arctan2(np.gradient(points[:, 1]), np.gradient(points[:, 0]))
    normal = np.c_[-np.sin(tangent), np.cos(tangent)]
    offset = np.zeros_like(s)
    objects = []
    for os in (620, 1830, 2920):
        pt = base.at(os)
        yaw = float(np.interp(os, base.s, base.yaw))
        objects.append([float(pt[0]), float(pt[1]), yaw, .8, .8, 0])
        up = np.clip((s - (os - 60)) / 42, 0, 1)
        down = np.clip(((os + 60) - s) / 42, 0, 1)
        smooth = lambda u: 10 * u**3 - 15 * u**4 + 6 * u**5
        offset += 3.5 * smooth(up) * smooth(down)
    route = Route(points + normal * offset[:, None])
    road = LineString(points).buffer(5.25, cap_style=2)
    # Extend start/end to account for body overhang.
    road = unary_union([road, LineString([points[0] - 8 * (points[1] - points[0]) /
                np.linalg.norm(points[1] - points[0]), points[1]]).buffer(5.25),
                LineString([points[-2], points[-1] + 8 * (points[-1] - points[-2]) /
                np.linalg.norm(points[-1] - points[-2])]).buffer(5.25)])
    lights = []
    for idx, bs in enumerate((320, 1370, 2570)):
        pt = base.at(bs)
        rs, _, _ = route.project(*pt, bs)
        lights.append({"s": rs, "period": 45, "red": 22, "offset": idx * 7,
                       "point": pt.tolist()})
    return Scenario("city", "城市连续道路 / 自建 3.68 km", [route], road,
                    (*route.points[0], route.yaw[0]), 12.5, 900, objects, lights,
                    [(0, 500, 11.11), (500, 1050, 13.88), (1050, 1650, 8.33),
                     (1650, 2350, 13.88), (2350, 2800, 11.11), (2800, 3800, 13.88)],
                    base=1000, official_example=True,
                    metadata={"nominal_route_length_m": 3680, "base_route": points[::10].tolist(),
                              "lane_changes": [[float(os - 60), float(os + 60)] for os in (620, 1830, 2920)]})


def make_scenario(key, vehicle):
    road = box(-8, 0, 150, 7)
    if key in ("traffic_red", "traffic_green"):
        fixed = "green" if key.endswith("green") else None
        light = {"s": 48, "period": 300, "red": 20, "point": [48, 1.75]}
        if fixed:
            light["fixed"] = fixed
        return Scenario(key, "信号灯 / " + ("红转绿" if not fixed else "绿灯通行"),
                        [straight(100)], road, (0, 1.75, 0), 7.5,
                        lights=[light], official_example=True)
    if key == "obstacle_start":
        return Scenario(key, "前方障碍物起步", [lane_change(start=.0, end=10.0)],
                        road, (0, 1.75, 0), 1.2,
                        obstacles=[[10.5, 1.75, 0, .45, .45, 0]], official_example=True,
                        metadata={"indicator_lead_s": 1.0})
    if key == "parking":
        road = unary_union([box(-8, .0, 55, 9.5), box(17.5, -6.0, 26.5, .2)])
        parked = [box(17.6, -5.9, 19.4, -1.3), box(24.6, -5.9, 26.4, -1.3)]
        segments, leaving, goal = parking_plan(vehicle, road, parked)
        obstacles = [[18.5, -3.6, math.pi / 2, 4.6, 1.8, 0],
                     [25.5, -3.6, math.pi / 2, 4.6, 1.8, 0]]
        return Scenario(key, "垂直泊车 / 倒入-停稳-驶出", segments + [leaving], road,
                        (0, 3.5, 0), .65, obstacles=obstacles,
                        slot=box(20.4, -6.0, 23.6, .0), parking_goal=goal,
                        dwell_after=len(segments) - 1, base=200, official_example=True)
    if key == "city":
        return make_city()
    if key == "lane_keep":
        x = np.linspace(0, 220, 1600)
        route = Route(np.c_[x, 3 * np.sin(x / 32)])
        delta = route.points[-1] - route.points[-2]
        delta /= np.linalg.norm(delta)
        start_delta = route.points[1] - route.points[0]
        start_delta /= np.linalg.norm(start_delta)
        road = LineString(np.vstack([route.points[0] - 7 * start_delta, route.points,
                                     route.points[-1] + 7 * delta])).buffer(1.75, cap_style=3)
        return Scenario(key, "附加测试 / 弯道车道保持", [route], road,
                        (0, 0, route.yaw[0]), 7, base=0)
    if key == "aeb":
        return Scenario(key, "附加测试 / 静止车辆紧急制动", [straight(110)], road,
                        (0, 1.75, 0), 11.1, obstacles=[[55, 1.75, 0, 4.6, 1.8, 0]],
                        base=0, metadata={"stop_test": True, "initial_speed": 11.1, "detection_range_m": 22})
    if key == "acc":
        return Scenario(key, "附加测试 / 同车道跟车", [straight(120)], road,
                        (0, 1.75, 0), 11.1,
                        obstacles=[[28, 1.75, 0, 4.6, 1.8, 4]], base=0)
    if key == "pedestrian":
        return Scenario(key, "附加测试 / 行人横穿让行", [straight(100)], road,
                        (0, 1.75, 0), 8,
                        obstacles=[[40, -4, math.pi / 2, .6, .6, 1.2]], base=0,
                        metadata={"pedestrian": True})
    raise KeyError(key)


KEYS = ["traffic_red", "traffic_green", "obstacle_start", "parking", "city",
        "lane_keep", "aeb", "acc", "pedestrian"]
