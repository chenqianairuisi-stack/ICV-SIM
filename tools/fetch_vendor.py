"""Fetch unmodified, MIT licensed PythonRobotics planning code."""
import hashlib
import json
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "vendor"
BASE = "https://raw.githubusercontent.com/AtsushiSakai/PythonRobotics/master/"
FILES = {
    "reeds_shepp_path_planning.py": "PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py",
    "utils/angle.py": "utils/angle.py",
    "LICENSE.PythonRobotics": "LICENSE",
}


def main():
    manifest = []
    for name, upstream in FILES.items():
        data = urllib.request.urlopen(BASE + upstream, timeout=60).read()
        path = ROOT / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        manifest.append({"file": name, "url": BASE + upstream,
                         "sha256": hashlib.sha256(data).hexdigest()})
    (ROOT / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print("Downloaded PythonRobotics planner and MIT license")


if __name__ == "__main__":
    main()
