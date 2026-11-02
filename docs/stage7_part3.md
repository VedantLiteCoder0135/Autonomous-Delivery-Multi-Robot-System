# Stage 7, Part 3: Integrated Python Navigation

This patch is **additive**. It leaves the existing Stage 1–6 runner, Blender bridge,
Stage 2 scheduler, README, architecture and environment files untouched.

## Included

- `src/aifr/simulation/dwa_delivery.py`: opt-in `DWAIntegratedRobot` and
  structured `TravelResult`/`ControlSample` outputs.
- `tests/test_stage7_integration.py`: integration and fail-closed tests.
- `tools/demo_stage7_integrated.py`: two-leg synthetic delivery with a
  slowly moving pedestrian; exports `outputs/stage7_integrated_trace.csv`.

## Installation

Install Stage 7 Parts 1 and 2 first, then extract this patch into the
repository root (`AIFR_Project`). From the root:

```bash
source .venv/bin/activate
python -m pytest -q tests/test_stage7_integration.py
python tools/demo_stage7_integrated.py
python -m pytest -q
```

## What is integrated

1. A* calculates a global route from the existing Stage 4 occupancy grid.
2. Simulated LiDAR scans moving pedestrian AABBs (the static map is already
   represented in the occupancy grid).
3. The Part 2 tracker estimates obstacles from scan ranges using EKF pose.
4. DWA receives the A* route, EKF pose, and **estimated** obstacles, then
   selects differential-drive velocity commands every 0.05 seconds.
5. Simulated odometry/landmarks update EKF; a separate ground-truth collision
   oracle rejects unsafe movement.
6. When obstacles are detected, a temporary A* overlay can provide a detour;
   the original static occupancy grid is never modified.

## Limitations / next step

- This is **offline, pure Python**, not live Blender navigation. Blender
  integration is the next part.
- The pedestrian sensor uses a simplified AABB raycaster and a basic tracker.
  No static-map LiDAR occlusion or robust data association yet.
- DWA can return `safe=False` when no feasible candidate exists. The runner
  **fails closed** instead of claiming a physically guaranteed emergency stop.
- The included demo uses ideal odometry/landmarks to isolate navigation
  integration. Realistic localization noise, crowds and narrow corridors still
  require validation.
- This patch does not alter FCFS, order lifecycle, fleet state, or base return.
- This is not proof that all Stage 7 requirements are complete.
