"""Re-run safety-sensitive cases at half the integration timestep."""
import json
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from icv.model import Vehicle
from icv.scenarios import make_scenario
from icv.simulation import run

results = []
for key in ("traffic_red", "traffic_green", "obstacle_start", "parking", "city", "lane_keep", "aeb", "acc", "pedestrian"):
    result, _ = run(make_scenario(key, Vehicle()), Vehicle(), .025)
    results.append(result)
    print(key, "PASS" if result["passed"] else "FAIL", result["time_s"], flush=True)
(ROOT / "output" / "half_timestep_checks.json").write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
if not all(r["passed"] for r in results):
    raise SystemExit(1)
