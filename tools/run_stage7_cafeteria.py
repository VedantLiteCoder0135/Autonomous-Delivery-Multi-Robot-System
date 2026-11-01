"""Run Stage 7 DWA on the *actual* extracted cafeteria grid (no Blender import).

    python tools/run_stage7_cafeteria.py
    python tools/run_stage7_cafeteria.py --no-pedestrian  # static baseline only
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT / "src"))

from stage7_cafeteria_core import run_cafeteria  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Stage 7 closed-loop cafeteria navigation")
    parser.add_argument("--no-pedestrian", action="store_true",
                        help="Run a static-map baseline (does not prove dynamic avoidance)")
    parser.add_argument("--max-ticks-per-leg", type=int, default=6000)
    parser.add_argument("--noise", type=float, default=0.0,
                        help="Odometry, yaw and landmark standard deviation (default 0)")
    args = parser.parse_args()
    try:
        report = run_cafeteria(PROJECT, with_pedestrian=not args.no_pedestrian,
                               max_ticks_per_leg=args.max_ticks_per_leg,
                               noise_sigma=args.noise)
    except (ValueError, FileNotFoundError, KeyError, TypeError) as exc:
        print(f"STAGE 7 CAFETERIA PRECHECK FAILED: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({key: report[key] for key in
                      ("success", "pickup", "delivery", "footprint_clear",
                       "lidar_detection_ticks", "trace_points")}, indent=2))
    if not report["success"]:
        print("STAGE 7 FAILED. Read outputs/stage7_cafeteria_report.json", file=sys.stderr)
        return 1
    print("STAGE 7 CAFETERIA PYTHON PASS: outputs/stage7_cafeteria_trace.json")
    print("Next: open Blender with tools/blender_stage7_bridge.py to create the visual demo.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
