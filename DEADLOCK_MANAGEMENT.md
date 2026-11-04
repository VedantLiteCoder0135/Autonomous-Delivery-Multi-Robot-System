# AIFR live traffic management (Blender-native)

This patch adds a **conservative shared-aisle traffic manager** for the integrated project. It is intentionally a safety-first interim approach, not a guarantee of deadlock freedom under every crowd configuration.

## Coordinated-mode deadlock handling (current)

`traffic_mode="coordinated"` (the Blender default) runs several robots at once. Deadlocks are
prevented and recovered without ever relaxing a collision check:

| Layer | Where | What it does |
|---|---|---|
| Reservation | `Stage79Engine.step`, `DeadlockManager.robot_pair_conflict` | Priority-ordered swept-disc check. Closing inside the soft envelope is vetoed, but motion that does *not* reduce the gap is allowed (bases are closer than the envelope, so the old rule froze the fleet). Physical footprint contact (+2 cm) is never allowed. Each veto is recorded as a wait-for edge. |
| Stuck detection | `_track_progress` | Progress = shorter remaining route or a clearly better heading. No progress for 4 s => `STUCK` (replaces the velocity-based stall counters and their FAULT timeouts). |
| Wait-for graph | `traffic_coordinator.WaitForGraph` | Cycles among stalled robots are detected; the lowest-priority robot (priority ages with every completed yield, so nobody starves) retreats. |
| Single-lane reservation | `traffic_coordinator.LaneMap/lane_claim` | A lower-priority robot does not enter a one-robot-wide aisle claimed head-on by a higher-priority one; if it is already in the other's lane it leaves to a passing bay first. Waived after 25 s so a stuck claimant cannot block forever. |
| Retreat / refuge | `_begin_retreat`, `find_refuge` | Drives (as a normal DWA-checked route) to the nearest wide bay clear of other robots, their next 4.5 m of route, and nearby people; holds with backoff, then resumes the *same* mission. Order, phase and scheduler state are untouched. |
| Recovery ladder | `_handle_stuck` | replan (A*/ALNS around traffic) -> yield/retreat -> longer backoff. Blocking never produces `FAULT`; only the exact-footprint, pedestrian and obstacle collision guards do. |
| People | `_identify_blocker`, `CustomerFlow` | Robots give way to pedestrians at once. Walkers meeting head-on: the later arrival walks to a passing bay and waits; the other holds its ground. |

Root causes fixed: (1) the reservation envelope (2x(radius+margin)) exceeded the base spacing, so every
move was vetoed with no escalation; (2) DWA let a robot inside a person's margin creep 5 cm per tick
toward contact, wedging robot and person together; (3) head-on robots/people in one-lane aisles had no
retreat behaviour; (4) the lidar ray-march scanned all obstacles (now pre-filtered, same output).

Diagnostics: `snapshot()["deadlock_stats"]` (`resolved`, `retreats`, `corridor_waits`, `stuck_events`) and
per-robot `wait_on`, `hold`, `retreating`, `yields`; the Blender panel shows them.

Tests: `tests/test_deadlock_scenarios.py` (`-m "not slow"` for the fast unit tests).

Known limits: coordination is centralised and reactive (no time-parametrised multi-agent path
planning); a corridor sealed for a long time leaves robots backing off and retrying rather than failing;
seated customers are permanent obstacles at their seat anchors.

## Implemented

- **Robot ↔ robot:** The live Blender engine runs with `traffic_mode="exclusive"`. At most **one robot** is admitted to active pickup / delivery / return travel at a time. Other assigned robots remain `WAITING_FOR_AISLE` and BUSY, preserving FCFS and return-before-reassignment. Once the active robot returns to base, the next is admitted. This prevents two robots entering the shared route head-on, at the cost of concurrency.
- **Customer ↔ robot:** Each proposed pedestrian step is checked against robot footprint envelopes plus 10 cm safety buffer. If blocked, the customer waits without entering the robot footprint and periodically attempts A* rerouting on a temporary occupancy grid with robot positions inflated. After a prolonged block, an event `CUSTOMER_TRAFFIC_BLOCKED` is recorded. No teleportation or collision-guard bypass.
- **Monitoring:** The Blender panel displays the conservative traffic mode and pedestrian-yield counter. Engine snapshots include traffic mode, pedestrian yields and reroute counts.
- **Compatibility:** Existing non-Blender simulation continues with `traffic_mode="standard"` (original two-robot maximum), preserving previous regression behavior. The recovered V14 scene is not modified by this patch.

## Installation (Git repository)

```bash
cd ~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System
git status  # Commit/back up local edits to the three files being replaced
unzip -o ~/Downloads/AIFR_Deadlock_Manager_Blender_Patch.zip -d .
source .venv/bin/activate
python -m pytest -q tests/test_deadlock_manager.py tests/test_customer_flow.py tests/test_live_runtime.py
python -m pytest -q
```

Start the native live Blender simulation (no server):

```bash
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" \
  --python "$(pwd)/tools/blender_integration_live.py"
```

In Blender select **N → AIFR**, add two robots and multiple customers, then Start. Only one robot travels at a time; the other can retain an assigned order in `WAITING_FOR_AISLE`. The panel shows pedestrian yields. The speed buttons still work.

## Verified in the development environment

- 24 targeted traffic, customer-flow and live-runtime tests passed.
- 2-robot / 3-order serialized scenario completed in 6,695 fixed ticks; return-before-reassignment true; no reported robot overlap.
- 1-customer / 2-robot scene-internal simulation completed payment, seating, delivery and robot return to base in 2,319 ticks.

## Limitations and next steps

- **Single robot per shared aisle is deliberately conservative.** It avoids robot-vs-robot face-to-face conflicts, but doesn't provide simultaneous corridor reservations or passing-bay negotiation.
- **Pedestrian movement is still kinematic.** It reroutes around known robot poses, but a completely blocked passage can force a customer to wait. The `CUSTOMER_TRAFFIC_BLOCKED` event reports such cases instead of claiming a solved route. Pedestrian-vs-pedestrian crowd avoidance is not handled here.
- If a robot is obstructed by a seated customer or an unreachable pedestrian destination, DWA can still stop and eventually declare a fault. Resolving this properly requires combined time-space planning and a safe refuge/queueing model.
- Reverify GUI motion in Blender on macOS; headless Python tests do **not** validate the Blender viewport.

Keep `outputs/`, `.venv/`, caches, and generated `.blend` files out of Git. Commit source, tests and documentation only.

## Persistent blocks and non-yielding patrols

Navigation now exposes `navigation_state` independently of the order phase. A
`BLOCKED` robot retains its order, custody links and fleet reservation. Four
unsuccessful recoveries trigger a connectivity check against static geometry and
loose obstacles, including alternative service waypoints. A sealed mission stops
planning until obstacle, map or patrol geometry changes. Replanning and waiting
cannot reset the mission recovery budget: twelve recovery events without mission
progress for 120 simulation seconds also stop automatic retries. The native
Blender panel and dashboard provide **Retry Blocked** for an operator-requested
retry after correcting traffic; it does not release or reassign the order.

Known patrol reservations survive the transient-agent routing fallback. Retreats
may leave the reserved patrol line and use acceleration-limited reverse motion,
but still pass DWA's obstacle prediction and braking checks, centralized robot
reservations, static footprint checks and swept collision guards. Local planning
uses current agents instead of retaining their old positions as permanent walls.
Pedestrian radii come from the actual agents, and pedestrian/robot clearance is
measured over the full simulation tick rather than only its endpoint.

Run deterministic regressions and stress tests with `.venv/bin/python -m pytest`.
They include four robots, 8 and 12 customers, narrow counter approaches, two
non-yielding patrols, sealed destinations, retained order custody and bounded
recovery. Stress scenarios require delivery and return within their tick limits.

Run the Blender smoke test without saving the scene:

```sh
/Applications/Blender.app/Contents/MacOS/Blender --background \
  AIFR_Cafeteria_V14_Recovered.blend --python tools/verify_blender_live.py
```

This checks actual Blender objects against engine robot/customer poses and blocked
state. Viewport redraw, sidebar interaction and timer responsiveness require a
manual Blender UI check.

The former `.gitignore` directory was replaced with an ignore file. Its virtual
environment was preserved at `.venv`, with launch scripts adjusted to that path.
Tracked environment and bytecode entries were removed from the Git index; source,
reference data, `.blend` scenes and existing tracked output artifacts are retained.
