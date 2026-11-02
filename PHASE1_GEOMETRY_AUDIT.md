# Phase 1: Cafeteria narrow-passage and pickup geometry audit

This is a **read-only diagnostic** for the recovered V14 cafeteria. It does not edit the `.blend` scene or change the live controller, robot routing, order ownership, or deadlock manager. Install the files into the **existing Git repository root**.

## Run from the repository root (Python 3.12 environment)

```bash
source .venv/bin/activate
python -m pytest -q tests/test_geometry_audit.py
python -m tools.run_geometry_audit
```

Inspect `outputs/phase1_geometry_audit.json`. Generated reports should stay out of Git. Confirm the reference exports exist under `data/reference_cafeteria/` before running. For regression protection:

```bash
python -m pytest -q
python tools/run_stage79.py
```

Do **not** replace your `tools/stage79_engine.py` or `blender_integration_live.py` with old copies. This patch adds only two Python modules, a test file and this document.

## What is actually checked

- Reports original food-counter positions (`TEA`, `SANDWICH`, `JUICE`, `MAIN_COURSE`) from the extracted scene.
- Lists raw opposing-face gaps to named nearby obstacle rectangles for geometry inspection. **A counter-wall gap is not necessarily a walkable aisle**; look at the object names and spatial context.
- Evaluates every existing pickup waypoint at its **required counter-facing heading**, using the exact oriented-rectangle collision checker (robot 0.40 m × 0.36 m; static obstacle margin 0.05 m).
- Runs A* reachability to each pickup, then tests collinear final approaches from behind it at several standoffs. The final approach motion is checked with the exact footprint checker.
- Checks whether a full in-place rotation at the pickup pose would be statically collision-free. Rotation is **diagnostic**, not required for a valid pickup.
- Computes residual clearance to already-inflated occupied grid cells along A* routes and within 2 m of each counter. **Do not interpret twice-inflated grid clearance as an actual measured aisle width.**
- Audits existing waiting-waypoint candidates for reachable positions and orientation clearance. Not being a traffic obstruction still requires additional corridor-level checks in Phase 4.

## Findings from the recovered V14 export (local Python audit)

| Pickup station | Candidates passing static pose + A* + straight-approach checks | Additional work |
| --- | ---: | --- |
| MAIN_COURSE (left-side counter) | 1/1 | Verify motion-controller approach/exit maneuvers through the tight left aisle |
| TEA | 1/1 | Verify turning and final approach while other agents are present |
| SANDWICH | 1/1 | Verify successive pickups and traffic priority |
| JUICE | 4/4 | Examine changing clearance along the long counter, especially end approaches |

These are **necessary geometry checks, not proof that DWA can physically negotiate the full route**. The previous left-side deadlock remains unresolved until narrow-space planning and corridor reservations are implemented and tested. The existing 5 waiting candidates are not yet certified as nonblocking for traffic.

## Next gate (before Phase 3/4)

1. Replay a real single-robot **MAIN_COURSE pickup, table delivery and return-to-base** in the existing engine, with no other agents; capture position/heading and DWA stop reasons.
2. Verify the counter-facing pose actually reached, not merely distance-to-point.
3. Test the same journey with one stationary agent at the aisle edge; classify feasible detour versus physically blocked passage.
4. Inspect the reported left-side counter/table gaps in Blender top view. Confirm whether the robot can turn at the pickup approach or needs a forward-in/reverse-out service maneuver.
5. Use these measurements to define the actual **single-file corridor polygon, entry threshold and outside waiting bay**. That belongs to Phase 4, not a premature hardcoded reservation.

The audit reports `CANDIDATE_FEASIBLE` only for the limited geometric subtests above, and expressly marks steering into the pickup approach as unverified.
