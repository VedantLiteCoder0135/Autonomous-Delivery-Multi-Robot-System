# AIFR live traffic management (Blender-native)

This patch adds a **conservative shared-aisle traffic manager** for the integrated project. It is intentionally a safety-first interim approach, not a guarantee of deadlock freedom under every crowd configuration.

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
