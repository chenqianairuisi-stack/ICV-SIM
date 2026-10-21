"""Run reproducible synthetic scenarios and export measured replay data."""
import argparse
import csv
import json
from pathlib import Path
from icv.model import Vehicle
from icv.scenarios import KEYS, make_scenario
from icv.simulation import run

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=["all"] + KEYS, default="all")
    parser.add_argument("--dt", type=float, default=.05)
    parser.add_argument("--backend", choices=("python", "cpp"), default="python")
    parser.add_argument("--native-library", help="Path to the CMake-built icv_native DLL")
    args = parser.parse_args()
    if not .01 <= args.dt <= .1:
        parser.error("--dt must be between 0.01 and 0.1 seconds")
    backend = None
    if args.backend == "cpp":
        from icv.native import NativeBackend
        backend = NativeBackend(args.native_library)
    elif args.native_library:
        parser.error("--native-library requires --backend cpp")
    out = ROOT / "output" / "cpp" if backend else ROOT / "output"
    out.mkdir(exist_ok=True)
    keys = KEYS if args.scenario == "all" else [args.scenario]
    vehicle = Vehicle()
    report, replays = [], []
    for key in keys:
        print("Running", key, flush=True)
        scenario = make_scenario(key, vehicle)
        result, trace = run(scenario, vehicle, args.dt, backend=backend)
        if backend:
            result["artifact_subdir"] = "cpp/"
        report.append(result)
        road = [list(scenario.road.exterior.coords)] if scenario.road.geom_type == "Polygon" else [list(p.exterior.coords) for p in scenario.road.geoms]
        replay = {"result": result, "trace": trace[::max(1, round(.1 / args.dt))],
                  "routes": [r.points[::max(1, len(r.points) // 1800)].tolist() + [r.points[-1].tolist()] for r in scenario.routes],
                  "road": road, "obstacles": scenario.obstacles, "lights": scenario.lights,
                  "slot": list(scenario.slot.exterior.coords) if scenario.slot is not None else None,
                  "vehicle": vars(vehicle), "metadata": scenario.metadata}
        replays.append(replay)
        with (out / (key + ".csv")).open("w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "x_m", "y_m", "yaw_rad", "speed_mps", "accel_mps2",
                             "steer_rad", "progress_m", "tracking_error_m", "segment", "indicator",
                             "target_speed_mps", "light_states", "speed_limit_mps"])
            writer.writerows(trace)
        (out / (key + ".json")).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(key, "PASS" if result["passed"] else "FAIL", "time", result["time_s"],
              "checks", result["checks"], flush=True)
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    replay_file, variable = ("replays_cpp.js", "REPLAYS_CPP") if backend else ("replays.js", "REPLAYS")
    (ROOT / "web" / replay_file).write_text("window." + variable + "=" + json.dumps(replays, ensure_ascii=False, separators=(",", ":")) + ";", encoding="utf-8")
    if backend:
        backend.close()
    if not all(r["passed"] for r in report):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
