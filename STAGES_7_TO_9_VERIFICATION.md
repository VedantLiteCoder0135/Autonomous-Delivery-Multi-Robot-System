# AIFR Stages 7–9: VS Code verification before Blender

This project retains the Stage 1–6 code and recovered cafeteria Blender scene, and adds a Python-only Stages 7–9 pipeline.

## What's implemented

- Stage 7: LiDAR obstacle conversion/tracking, EKF-informed navigation, A*/DWA route following, static/dynamic obstacle safety checks, and a validated cafeteria delivery path.
- Entrance-mapping fix: four named V14 doors are classified from recovered custom metadata and verified against corresponding physical wall gaps, rather than assuming labels imply openings. These are geometry-classified entrances; their center cells need not be traversable in the conservatively inflated grid.
- Stage 8: ALNS route optimization (shortening collision-free geometric route segments, not globally solving VRP), multiple robot instances, narrow-aisle traffic admission and yielding, and return to base before availability/reassignment. Scheduler default remains backward-compatible; the integrated engine enables the return lifecycle explicitly.
- Stage 9: local-browser Python dashboard with order creation, map, robot statuses, pause/resume, step, speed, emergency stop, reset, save report. Does not use Blender.

## Install and verify (Mac, from project root)

```bash
cd ~/Downloads/AIFR_Project
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
python -m pytest -q
python tools/run_stage79.py
```

Expected default scenario: 2 robots, 3 delivered orders, 4 entrances, both robots returned to base and available, no reported collisions. Outputs: `outputs/stage79_report.json`, `outputs/stage79_trace.json`.

For one moving-pedestrian scenario (sensor-triggered trajectory differences compared with baseline):

```bash
python tools/run_stage79.py --robots 1 --orders T11:SANDWICH --pedestrian
```

For the local dashboard:

```bash
python tools/stage79_dashboard.py
```

Open the localhost address displayed by the server in your browser, then add orders and press Start. No Blender required.

## Known limitations

- The robot motion is discrete, kinematic and 2D, with conservative checks; it is not a physical motor/controller validation.
- ALNS is a path-segment optimizer, not a multi-vehicle ALNS assignment/VRP formulation. Assignment remains READY-time FCFS.
- The multi-robot traffic controller uses conservative narrow-aisle admission. It may reduce concurrency, especially with 3–4 robots.
- The entrance classifications validate scene geometry, but do not prove that robots can drive through all four entrance grid cells.
- The Stage 9 local dashboard was exercised with HTTP integration tests; the user should still inspect layout and interactions in their own browser.
- Do not run Blender bridge scripts yet. Finish checking these Python results first.
