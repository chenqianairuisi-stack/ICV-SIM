"""NEVC 3.0/3.1 ctypes adapter; requires a scenario-specific calibrated route."""
import argparse
import importlib
import json
import math
import os
from pathlib import Path
import socket
import sys
import time

from icv.model import Vehicle, State
from icv.planning import Route
from icv.controller import steering_control, speed_control, stopping_speed, obstacle_speed


def load_sdk(path):
    path = Path(path).resolve()
    sys.path.insert(0, str(path))
    os.chdir(path)
    handles = [os.add_dll_directory(str(path))] if hasattr(os, "add_dll_directory") else []
    struct = importlib.import_module("SimOneIOStruct")
    if not struct.LoadDllSuccess:
        raise RuntimeError("SDK DLL loading failed")
    return struct, importlib.import_module("SimOneServiceAPI"), importlib.import_module("SimOneSensorAPI"), importlib.import_module("SimOnePNCAPI"), handles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", default="E:/Sim-One/SimOneAPI/lib/Win64")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23789)
    parser.add_argument("--vehicle", default="0")
    parser.add_argument("--probe", action="store_true", help="Load DLL and report version without connecting")
    parser.add_argument("--config", help="Real scenario configuration, never an offline coordinate file")
    parser.add_argument("--seconds", type=float, default=300)
    args = parser.parse_args()
    config_path = Path(args.config).resolve() if args.config else None
    S, service, sensor, pnc, handles = load_sdk(args.sdk)
    print("SDK version:", service.SoAPIGetVersion())
    if args.probe:
        return
    if config_path is None:
        parser.error("A calibrated --config is required before controlling a real scenario")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if not config.get("calibrated_for_nevc"):
        parser.error("Set calibrated_for_nevc only after measuring actual NEVC coordinates and steering")
    route = Route(config["route_xy"])
    steering_scale = float(config["steering_radians_to_percent"])
    if steering_scale == 0:
        parser.error("steering_radians_to_percent must be calibrated and nonzero")
    source = config["obstacle_source"]
    if source not in ("sensor", "ground_truth"):
        parser.error("obstacle_source must be sensor or ground_truth")
    if source == "ground_truth" and not config.get("ground_truth_permitted_by_rules"):
        parser.error("Ground truth requires explicit rules permission in config")
    with socket.create_connection((args.host, args.port), timeout=3):
        pass
    if not service.SoInitSimOneAPI(args.vehicle, 0, args.host, args.port):
        raise RuntimeError("NEVC initialization failed")
    vehicle = Vehicle(**config.get("vehicle", {}))
    progress, last_frame, stale_since, start = 0., None, None, time.monotonic()
    gps, drive, signals = S.SimOne_Data_Gps(), S.SimOne_Data_Control(), S.SimOne_Data_Signal_Lights()
    obstacle_data = S.SimOne_Data_SensorDetections() if source == "sensor" else S.SimOne_Data_Obstacle()
    pnc.SoSetDriverName(args.vehicle, "ICV_Offline_Transfer_Baseline")

    def send(acceleration, steering, v, indicator=0):
        drive.EThrottleMode = 0
        drive.EBrakeMode = 0
        drive.ESteeringMode = 0
        drive.throttle = min(.5, max(0, acceleration / vehicle.accel))
        drive.brake = min(1., max(0, -acceleration / vehicle.brake))
        drive.steering = max(-1., min(1., steering * steering_scale))
        drive.isManualGear = False
        drive.gear = 1
        if abs(v) < .05 and acceleration <= 0:
            drive.throttle, drive.brake = 0, .35
        signals.signalLights = indicator
        if not pnc.SoSetSignalLights(args.vehicle, signals) or not pnc.SoSetDrive(args.vehicle, drive):
            raise RuntimeError("NEVC rejected a control command")

    try:
        while time.monotonic() - start < args.seconds:
            if service.SoGetCaseRunStatus() == 1:
                break
            if not sensor.SoGetGps(args.vehicle, gps) or gps.isGPSLost:
                send(-vehicle.brake, 0, 0)
                raise RuntimeError("GPS unavailable; brake requested")
            if gps.frame == last_frame:
                stale_since = stale_since or time.monotonic()
                if time.monotonic() - stale_since > .5:
                    send(-vehicle.brake, 0, 0)
                    raise RuntimeError("Stale GPS; brake requested")
                time.sleep(.01)
                continue
            stale_since, last_frame = None, gps.frame
            v = math.hypot(gps.velX, gps.velY)
            state = State(gps.posX, gps.posY, gps.oriZ, v)
            progress, error, tangent = route.project(state.x, state.y, progress)
            if abs(error) > config.get("max_route_error_m", 1.2):
                send(-vehicle.brake, 0, v)
                raise RuntimeError("Route mismatch; brake requested")
            if source == "sensor":
                ok = sensor.SoGetSensorDetections(args.vehicle, config["sensor_id"], obstacle_data)
                entries = obstacle_data.objects[:obstacle_data.objectSize]
            else:
                ok = sensor.SoGetGroundTruth(args.vehicle, obstacle_data)
                entries = obstacle_data.obstacle[:obstacle_data.obstacleSize]
            if not ok:
                send(-vehicle.brake, 0, v)
                raise RuntimeError("Obstacle input unavailable; brake requested")
            objects = [[o.posX, o.posY, o.oriZ, o.length, o.width, math.hypot(o.velX, o.velY)] for o in entries]
            target = min(config.get("cruise_mps", 6), stopping_speed(route.length - progress))
            limit, emergency, _ = obstacle_speed(state, route, progress, objects, vehicle)
            target = min(target, limit)
            for zone in config.get("speed_zones", []):
                if zone[0] <= progress < zone[1]:
                    target = min(target, zone[2] * .95)
            for light in config.get("lights", []):
                gap = light["stop_s"] - progress - vehicle.front
                if gap >= 0:
                    data = S.SimOne_Data_TrafficLight()
                    if not sensor.SoGetTrafficLights(args.vehicle, int(light["opendrive_id"]), data):
                        send(-vehicle.brake, 0, v)
                        raise RuntimeError("Traffic light input unavailable")
                    if data.status.value != 2:
                        target = min(target, stopping_speed(gap - .3))
            indicator = 0
            for zone in config.get("indicator_zones", []):
                if zone[0] <= progress < zone[1]:
                    indicator = int(zone[2])
            send(speed_control(state, target, 1, vehicle, emergency),
                 steering_control(state, route, progress, vehicle), v, indicator)
            if route.length - progress < .1 and v < .05:
                break
            time.sleep(.01)
    finally:
        try:
            send(-vehicle.brake, 0, 0)
        finally:
            service.SoTerminateSimOneAPI()


if __name__ == "__main__":
    main()
