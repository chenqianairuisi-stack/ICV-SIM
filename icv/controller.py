"""Shared perception-to-actuation logic for offline and NEVC backends."""
import math
import numpy as np
from .model import wrap


def steering_control(state, route, progress, vehicle):
    lookahead = min(8.0, .65 + abs(state.v) * .65)
    if route.direction < 0:
        lookahead = min(1.5, .6 + abs(state.v) * .45)
    target = route.at(progress + lookahead)
    alpha = wrap(math.atan2(target[1] - state.y, target[0] - state.x) -
                 state.yaw - (math.pi if route.direction < 0 else 0))
    distance = max(.3, math.hypot(target[0] - state.x, target[1] - state.y))
    delta = math.atan2(route.direction * 2 * vehicle.wheelbase * math.sin(alpha), distance)
    return float(np.clip(delta, -vehicle.max_steer, vehicle.max_steer))


def speed_control(state, desired, direction, vehicle, emergency=False):
    target = desired * direction
    effort = 2.5 * (target - state.v)
    if emergency:
        effort = -math.copysign(vehicle.brake, state.v) if abs(state.v) > .01 else 0
    # Reverse acceleration uses symmetric actuator limits in the offline model.
    return float(np.clip(effort, -vehicle.brake, vehicle.accel))


def stopping_speed(distance, deceleration=1.8):
    # A low-speed proportional approach avoids a residual gap from asymptotic braking.
    d = max(0.0, distance)
    return min(math.sqrt(2 * min(deceleration, .9) * d), .85 * d)


def obstacle_speed(state, route, progress, obstacles, vehicle):
    limit, emergency, gap = float("inf"), False, float("inf")
    for item in obstacles:
        x, y, yaw, length, width, velocity = item
        os, lateral, tangent = route.project(x, y, progress)
        if os >= route.length - .01:
            end = route.points[-1]
            if (x - end[0]) * math.cos(tangent) + (y - end[1]) * math.sin(tangent) > length / 2 + vehicle.front:
                continue
        ahead = os - progress
        if ahead <= 0 or abs(lateral) > vehicle.width / 2 + width / 2 + .25:
            continue
        # Ignore distant objects outside the planned corridor through a curve.
        if ahead > 90:
            continue
        clearance = ahead - vehicle.front - length / 2
        gap = min(gap, clearance)
        closing = max(0, abs(state.v) - velocity)
        safe = 2.5 + abs(state.v) * 1.3
        limit = min(limit, max(0, velocity + (clearance - safe) * .65))
        if clearance < max(1.0, closing * .35 + closing * closing / (2 * vehicle.brake) + .6):
            emergency = True
    return limit, emergency, gap
