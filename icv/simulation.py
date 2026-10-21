"""Closed-loop simulation and independent geometry/event measurements."""
import math
import numpy as np
from shapely import prepare
from shapely.geometry import box
from shapely.affinity import rotate, translate
from .model import State, advance
from .controller import steering_control, speed_control, stopping_speed, obstacle_speed
from .scenarios import light_state


def run(scenario, vehicle, dt=.05):
    state = State(*map(float, scenario.initial), scenario.metadata.get("initial_speed", 0))
    t, index, progress = 0., 0, 0.
    dwell_start, parked_seconds, parked_ok, parked_reverse = None, 0., False, False
    trace, events, stop_gaps, green_delays = [], [], [], []
    collision, outside, red_violation, speeding, unsignaled = False, False, False, False, False
    max_error, max_acceleration, min_clearance = 0., 0., float("inf")
    reached, hold_for_aeb, last_direction = False, None, 1
    signal_stop = {}
    last_light = {}
    crossing = set()
    lane_signal_start = None
    previous_road = False
    road_guard = scenario.road.buffer(.005)
    prepare(road_guard)

    while t <= scenario.timeout:
        route = scenario.routes[index]
        progress, error, tangent = route.project(state.x, state.y, progress)
        remaining = route.length - progress
        objects = scenario.objects(t)
        body = vehicle.polygon(state.x, state.y, state.yaw)
        in_road = road_guard.covers(body)
        if not in_road and not previous_road:
            events.append({"t": round(t, 3), "kind": "road_boundary"})
        outside |= not in_road
        previous_road = not in_road
        for item in objects:
            obstacle = translate(rotate(box(-item[3] / 2, -item[4] / 2,
                                             item[3] / 2, item[4] / 2),
                                        item[2], use_radians=True), item[0], item[1])
            min_clearance = min(min_clearance, body.distance(obstacle))
            if body.intersects(obstacle):
                collision = True
        if collision:
            events.append({"t": round(t, 3), "kind": "collision"})
            break

        max_error = max(max_error, abs(error))
        target = min(scenario.speed, stopping_speed(remaining, 1.3))
        # Curvature and upcoming speed limits are previewed before entering a zone.
        lo = np.searchsorted(route.s, progress)
        hi = min(len(route.s), np.searchsorted(route.s, progress + 25) + 1)
        curvature = np.max(np.abs(route.curvature[lo:hi])) if hi > lo else 0
        target = min(target, math.sqrt(1.5 / max(curvature, 1e-5)))
        current_limit = scenario.speed
        for start, end, speed in scenario.limits:
            if start <= progress < end:
                current_limit = speed
                target = min(target, speed * .98)
            elif progress < start:
                target = min(target, math.sqrt((speed * .98)**2 + 2 * 1.7 * max(0, start - progress - 5)))
        speeding |= abs(state.v) > current_limit + .10

        indicator = 0
        if scenario.key == "obstacle_start":
            indicator = 2 if progress < 13 else 0
        if scenario.key == "city":
            for start, end in scenario.metadata["lane_changes"]:
                if start - 15 <= progress < start + 45:
                    indicator = 2
                if end - 45 <= progress < end + 10:
                    indicator = 1
        if indicator and lane_signal_start is None:
            lane_signal_start = t
        if not indicator:
            lane_signal_start = None
        if scenario.key == "obstacle_start" and t < scenario.metadata["indicator_lead_s"]:
            target = 0

        light_colors = []
        for n, light in enumerate(scenario.lights):
            color = light_state(light, t)
            light_colors.append(color)
            gap = light["s"] - progress - vehicle.front
            if last_light.get(n) == "red" and color == "green" and n in signal_stop:
                signal_stop[n]["green"] = t
            last_light[n] = color
            if gap >= -.1 and color != "green":
                target = min(target, stopping_speed(gap - .3))
                if abs(state.v) < .03 and gap < 1.5 and n not in signal_stop:
                    signal_stop[n] = {"gap": gap, "t": t}
                    stop_gaps.append(float(gap))
                    events.append({"t": round(t, 3), "kind": "red_stop", "gap_m": round(gap, 4)})
            if n in signal_stop and "green" in signal_stop[n] and not signal_stop[n].get("started") and abs(state.v) > .1:
                delay = t - signal_stop[n]["green"]
                green_delays.append(delay)
                signal_stop[n]["started"] = True
            if gap < 0 and n not in crossing:
                crossing.add(n)
                if color != "green":
                    red_violation = True

        detected = objects
        if "detection_range_m" in scenario.metadata:
            detected = [o for o in objects if math.hypot(o[0] - state.x, o[1] - state.y) < scenario.metadata["detection_range_m"]]
        obstacle_limit, emergency, gap = obstacle_speed(state, route, progress, detected, vehicle)
        # Crossing prediction extends the obstacle corridor through the stopping horizon.
        if scenario.metadata.get("pedestrian"):
            for ox, oy, oyaw, length, width, velocity in objects:
                os, lateral, _ = route.project(ox, oy, progress)
                crossing_time = max(0., (os - progress - vehicle.front) / max(abs(state.v), 1))
                future_y = oy + math.sin(oyaw) * velocity * crossing_time
                _, future_lateral, _ = route.project(ox, future_y, progress)
                if os > progress and min(lateral, future_lateral) < 1.3 and max(lateral, future_lateral) > -1.3:
                    obstacle_limit = min(obstacle_limit, stopping_speed(os - progress - vehicle.front - 2.0))
        if scenario.key != "parking":
            target = min(target, obstacle_limit)
        if emergency:
            events.append({"t": round(t, 3), "kind": "aeb"}) if not events or events[-1]["kind"] != "aeb" else None
        if scenario.metadata.get("stop_test") and abs(state.v) < .03 and gap < 8:
            hold_for_aeb = t if hold_for_aeb is None else hold_for_aeb
            target = 0
            if t - hold_for_aeb >= 2:
                reached = True
                break

        if remaining < .10 and abs(state.v) < .04:
            if index == scenario.dwell_after:
                at_slot = scenario.slot.covers(body)
                gx, gy, gyaw = scenario.parking_goal
                heading_error = abs((state.yaw - gyaw + math.pi) % (2 * math.pi) - math.pi)
                parked_ok = bool(at_slot and heading_error < math.radians(8))
                parked_reverse = route.direction == -1
                if not parked_ok:
                    events.append({"t": round(t, 3), "kind": "parking_pose_invalid"})
                    break
                if dwell_start is None:
                    dwell_start = t
                    events.append({"t": round(t, 3), "kind": "parking_dwell_start"})
                parked_seconds = t - dwell_start
                target = 0
                if parked_seconds >= scenario.dwell:
                    index += 1
                    progress = 0
                    events.append({"t": round(t, 3), "kind": "parking_exit"})
                    continue
            elif index + 1 < len(scenario.routes):
                index += 1
                progress = 0
                continue
            else:
                reached = True
                break

        steering = steering_control(state, route, progress, vehicle)
        if target < .01 and abs(state.v) < .04:
            steering = 0
        acceleration = speed_control(state, target, route.direction, vehicle, emergency)
        next_state = advance(state, acceleration, steering, dt, vehicle)
        lateral_a = state.v**2 * math.tan(next_state.steer) / vehicle.wheelbase
        max_acceleration = max(max_acceleration, math.hypot(next_state.a, lateral_a))
        trace.append([round(t, 3), state.x, state.y, state.yaw, state.v,
                      next_state.a, next_state.steer, progress, error, index,
                      indicator, target, light_colors, current_limit])
        state = next_state
        t += dt

    result = {
        "key": scenario.key, "title": scenario.title, "backend": "offline_scipy_bicycle",
        "official_score": None, "official_example": scenario.official_example,
        "completed": reached, "time_s": round(t, 3), "collision": collision,
        "boundary_contact": outside, "red_violation": red_violation, "speeding": bool(speeding),
        "max_tracking_error_m": round(max_error, 4),
        "max_acceleration_mps2": round(max_acceleration, 4),
        "min_clearance_m": round(min_clearance, 4) if math.isfinite(min_clearance) else None,
        "red_stop_gaps_m": [round(g, 4) for g in stop_gaps],
        "green_start_delays_s": [round(d, 3) for d in green_delays],
        "parked_seconds": round(parked_seconds, 3), "parked_in_slot": parked_ok,
        "reverse_entry": parked_reverse, "progress_m": round(progress, 3),
        "route_length_m": round(sum(r.length for r in scenario.routes), 3),
        "events": events, "dt_s": dt,
    }
    result["indicator_before_change"] = (scenario.key != "obstacle_start" or
        all(row[10] == 2 for row in trace if .01 < row[7] < 8))
    result["local_score_estimate"], result["checks"] = score(scenario, result)
    result["passed"] = all(result["checks"].values())
    return result, trace


def score(scenario, r):
    checks = {"completed": r["completed"], "no_collision": not r["collision"],
              "inside_road": not r["boundary_contact"], "within_time": r["time_s"] <= scenario.timeout}
    estimate = scenario.base
    if not r["completed"] or r["collision"]:
        estimate = 0
    if r["boundary_contact"]:
        estimate -= 50
    if scenario.key.startswith("traffic"):
        checks.update({"no_red_crossing": not r["red_violation"],
                       "red_stop_within_0_5m": all(0 <= g <= .5 for g in r["red_stop_gaps_m"]),
                       "green_start_within_5s": all(d <= 5 for d in r["green_start_delays_s"])})
        if scenario.key == "traffic_red":
            checks["red_stop_observed"] = len(r["red_stop_gaps_m"]) > 0
        if (r["red_violation"] or any(g > 1 or g < 0 for g in r["red_stop_gaps_m"])
                or any(d > 5 for d in r["green_start_delays_s"])
                or (scenario.key == "traffic_red" and not r["red_stop_gaps_m"])):
            estimate = 0
        elif any(g > .5 for g in r["red_stop_gaps_m"]):
            estimate -= 50
    if scenario.key == "obstacle_start":
        checks["indicator_before_change"] = r["indicator_before_change"]
        if not r["indicator_before_change"]:
            estimate -= 50
    if scenario.key == "parking":
        checks.update({"inside_target_slot": r["parked_in_slot"], "reverse_entry": r["reverse_entry"],
                       "stationary_at_least_10s": r["parked_seconds"] >= 10})
        if not all(checks.values()):
            estimate = 0
        elif r["time_s"] < 120:
            estimate += 10 * int((120 - r["time_s"]) // 10)
    if scenario.key == "city":
        checks.update({"within_speed_limit": not r["speeding"],
                       "no_red_crossing": not r["red_violation"],
                       "acceleration_at_most_5": r["max_acceleration_mps2"] <= 5})
        estimate = 1000 if r["completed"] else 1000 * min(1, r["progress_m"] / scenario.routes[0].length)
        estimate -= 200 * r["speeding"] + 200 * r["red_violation"]
        if r["max_acceleration_mps2"] <= 5:
            estimate += 200
        if r["completed"] and r["time_s"] < 420:
            estimate += 5 * int(420 - r["time_s"])
    if scenario.base == 0:
        estimate = None
    if scenario.key == "aeb":
        checks["emergency_brake_observed"] = any(e["kind"] == "aeb" for e in r["events"])
    return (round(max(0, estimate), 1) if estimate is not None else None), checks
