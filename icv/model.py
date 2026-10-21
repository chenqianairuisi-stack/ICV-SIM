"""Rear-axle bicycle dynamics, integrated by SciPy RK45 in SI units."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.integrate import solve_ivp
from shapely.geometry import Polygon


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Vehicle:
    wheelbase: float = 2.8
    front: float = 3.6
    rear: float = 1.0
    width: float = 1.8
    max_steer: float = math.radians(32)
    max_steer_rate: float = math.radians(80)
    accel: float = 2.4
    brake: float = 4.5

    def polygon(self, x, y, yaw, margin=0.0):
        points = np.array([[self.front + margin, self.width / 2 + margin],
                           [self.front + margin, -self.width / 2 - margin],
                           [-self.rear - margin, -self.width / 2 - margin],
                           [-self.rear - margin, self.width / 2 + margin]])
        c, s = math.cos(yaw), math.sin(yaw)
        return Polygon(points @ np.array([[c, s], [-s, c]]) + [x, y])


@dataclass
class State:
    x: float
    y: float
    yaw: float
    v: float = 0.0
    steer: float = 0.0
    a: float = 0.0


def advance(state, acceleration, steering, dt, vehicle):
    a = float(np.clip(acceleration, -vehicle.brake, vehicle.accel))
    delta = float(np.clip(steering, -vehicle.max_steer, vehicle.max_steer))
    delta = float(np.clip(delta, state.steer - vehicle.max_steer_rate * dt,
                          state.steer + vehicle.max_steer_rate * dt))
    # Low-speed friction prevents a braking command from changing direction.
    if state.v * (state.v + a * dt) < 0:
        a = -state.v / dt

    def ode(_, z):
        _, _, yaw, v = z
        return [v * math.cos(yaw), v * math.sin(yaw),
                v * math.tan(delta) / vehicle.wheelbase, a]

    z = solve_ivp(ode, (0, dt), [state.x, state.y, state.yaw, state.v],
                  rtol=1e-7, atol=1e-9).y[:, -1]
    if abs(z[3]) < 1e-7:
        z[3] = 0.0
    return State(float(z[0]), float(z[1]), wrap(z[2]), float(z[3]), delta, a)
