# Stage 7, Part 4: Actual Cafeteria DWA Simulation + Blender Playback

This patch **adds** an opt-in closed-loop navigation run against the **real Stage 3/4 exports** and an independent Blender visualization bridge. It does not replace the existing Stage 1–6 bridge or alter the approved cafeteria scene.

## Prerequisites

- Stage 7 Parts 1–3 installed in the same `AIFR_Project` folder.
- A successful run of `tools/blender_stage16_bridge.py` (the existing Stages 1–6 bridge) on the approved `AIFR_Simulation_Stage3.blend`.
- These files already present in `outputs/`: `stage36_report.json`, `stage3_environment.json`, `stage4_occupancy_grid.json`.
- Blender installed on macOS. The bridge is intended for the Blender 5.x scene; it has **not been executed in Blender in the packaging environment**.

## Files in this patch

| File | Purpose |
|---|---|
| `tools/stage7_cafeteria_core.py` | Loads real scene exports, validates mission, chooses safe pedestrian location, runs A* + EKF + LiDAR + DWA and writes reports |
| `tools/run_stage7_cafeteria.py` | Command-line entry point for the Python simulation |
| `tools/blender_stage7_bridge.py` | Animates the verified trace in a separate Blender copy |
| `tests/test_stage7_cafeteria.py` | Pure-Python checks for exports, pedestrian placement, navigation, and fail-closed behavior |
| `docs/stage7_part4.md` | This guide |

## Step 1: Add the patch

From the project root on macOS:

```bash
cd ~/Downloads/AIFR_Project
source .venv/bin/activate
unzip -o ~/Downloads/Stage7_Part4_Cafeteria_Blender.zip -d .
```

## Step 2: Verify the Python code

```bash
python -m pytest -q tests/test_stage7_cafeteria.py
python -m pytest -q
```

## Step 3: Run on the actual cafeteria map

```bash
python tools/run_stage7_cafeteria.py
```

This runs the actual validated **spawn → Sandwich counter → T11 table** mission with a moving pedestrian selected from free space near the delivery route. It reuses the **existing** Stage 4 grid and exact footprint checker. The Python simulator makes velocity decisions every 0.05 s; it does **not** use a hand-authored animation.

The command writes:

- `outputs/stage7_cafeteria_report.json`: mission success/failure, LiDAR detections, collision checks, and comparison with a no-pedestrian baseline.
- `outputs/stage7_cafeteria_trace.json`: robot pose, EKF pose, velocity commands, pedestrian position and planned paths for Blender playback.

The run is considered successful only if **both legs reach their destinations**, the recorded footprint is clear, the independent dynamic guard remains clear, and LiDAR detects the pedestrian. The report's `baseline_comparison.lidar_affected_motion` separately indicates whether the pedestrian *measurably changed* robot movement; detection alone does not establish avoidance.

If this command fails, **do not run the Blender bridge yet**. Inspect the JSON report. To isolate map/navigation problems from pedestrian problems, run the static baseline:

```bash
python tools/run_stage7_cafeteria.py --no-pedestrian
```

**Warning:** this overwrites the Stage 7 report and trace with a **static-only** run. It does not count as dynamic-obstacle avoidance. Rerun the default command to restore the dynamic trace.

If your scene was moved or modified after the Stage 1–6 export, rerun the original Blender bridge first to regenerate the authoritative JSON files.

## Step 4: Create the Blender demonstration

Only after the default Python run reports `"success": true`:

```bash
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Simulation_Stage3.blend" \
  --background \
  --python "$(pwd)/tools/blender_stage7_bridge.py"
```

The bridge creates **`outputs/AIFR_Stage7_Demo.blend`** and **`outputs/stage7_blender_report.json`**. It will not save over `AIFR_Simulation_Stage3.blend`. It verifies the original file's SHA-256 digest before and after saving.

Open the demo in Blender:

```bash
open -a Blender "$(pwd)/outputs/AIFR_Stage7_Demo.blend"
```

Press **Space** on the Blender timeline to play. The demo collection `AIFR_STAGE7_DEMO` contains the animated robot, animated pedestrian, green actual trace, and separate A* routes for pickup/delivery.

## What this does *not* prove

- **Blender playback is not real-time Blender physics.** The 20 Hz navigation decisions happen in Python first.
- The simulated LiDAR currently sees pedestrian AABBs, not occlusion by cafeteria walls or furniture. Static obstacles are handled by the occupancy grid and footprint checker.
- The test scenario has **one robot and one pedestrian**, not the Stage 8 multi-robot fleet.
- The simulation defaults to ideal odometry and landmarks to isolate navigation. You can try `--noise 0.01` later, but noisy operation requires its own validation.
- Stage 2 FCFS is not re-executed here. The waypoints are reused from its earlier verified delivery.
- Even a successful run is not proof of robustness in crowds, blocked narrow aisles, or every pickup/table pair.

**Do not mark Stage 7 fully complete solely because the Blender animation looks smooth.** Review the reports, including whether the moving pedestrian affected the controls, and test harder dynamic-obstacle scenarios.
