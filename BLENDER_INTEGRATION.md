# Stage 10, Step 1: Stages 7–9 Blender playback in the GitHub repository

This is the **first Blender integration**, not live interaction. Python simulates the
robots, A*/ALNS/DWA/EKF/LiDAR/FCFS/return-to-base; Blender plays back the
validated robot trajectories in the recovered V14 cafeteria. It **does not**
re-run navigation or accept orders from Blender. Do not describe this as
Blender physics-based control or a full interactive order dashboard.

## 1. Copy files into your existing Git checkout

From the directory containing your project's `.git` folder (for example,
`~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System`), copy only:

- `tools/stage79_blender_prep.py`
- `tools/blender_stage79_bridge.py`
- `tests/test_stage79_blender_prep.py`
- `STAGE79_BLENDER_INTEGRATION.md`

**Do not copy a new `.git` folder and do not overwrite the original V14 `.blend`.**
The bridge needs `AIFR_Cafeteria_V14_Recovered.blend` at the project root.
If your source is nested under `AIFR_Project/`, run the commands from that
nested project root, but keep Git operations in the Git checkout.

## 2. Python verification in VS Code

```bash
pwd
ls -la .git   # if the Git repo root is also the Python project root
ls AIFR_Cafeteria_V14_Recovered.blend tools/run_stage79.py
source .venv/bin/activate
python -m pytest -q
python tools/run_stage79.py
python -m pytest -q tests/test_stage79_blender_prep.py
```

The simulation must print `"success": true`. The bridge will reject missing,
failed, incomplete, or mismatched reports and traces. Never visualize a failed
run as if it succeeded. Check `outputs/stage79_report.json` first.

## 3. Generate a Blender animation from the Git checkout

```bash
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" \
  --background \
  --python "$(pwd)/tools/blender_stage79_bridge.py"
```

Expected outputs:

- `outputs/AIFR_Stages_7_9_Demo.blend` (new file, never overwrites V14)
- `outputs/stage79_blender_report.json` (success, source scene preserved, keyframes)

Open the animation:

```bash
open -a Blender "$(pwd)/outputs/AIFR_Stages_7_9_Demo.blend"
```

Go to Layout and press Space to play. Two colored 3-D robot markers, route
curves, actual paths, and four entrance markers are added in the
`AIFR_STAGE79_VISUALIZATION` collection. Timeline markers identify lifecycle
events. At 30 fps, the default 227.5 seconds runs for roughly 6,826 frames;
use the timeline to navigate. For a short demo, adjust playback range in a
**copy of the output .blend**, not in the original source.

## 4. Verify the output

```bash
cat outputs/stage79_blender_report.json
ls -lh outputs/AIFR_Stages_7_9_Demo.blend
```

Look for `"success": true`, `"original_scene_unchanged": true`,
`"entrances": 4`, two robot entries, and 3 orders delivered. The source
Blender cafeteria is never saved by the bridge.

## 5. Commit only code and documentation

```bash
git status --short
git add tools/stage79_blender_prep.py tools/blender_stage79_bridge.py \
  tests/test_stage79_blender_prep.py STAGE79_BLENDER_INTEGRATION.md
git diff --cached --stat
git commit -m "Add Stage 7-9 Blender trace visualization bridge"
```

Push using your normal branch / pull-request process. Generated `.blend`
animations, JSON trace files and `.venv` should stay out of Git unless your
team explicitly decides to version them.

### Limitations / next work

The bridge visualizes the *already verified* 1–4-robot Python outputs. It does
not animate pedestrians because the current trace does not export their
positions. The default 2-robot run has no pedestrians. For live pedestrian
visualization, add timestamped pedestrian samples to the engine export before
implementing the corresponding Blender meshes. Interactive orders/robot
spawning in Blender are a separate later integration step.
