# AIFR Live Blender Integration (Git repository)

This is the **live controller**, separate from the existing offline animation exporter.
The recovered V14 cafeteria remains authoritative. All simulation decisions run
in the existing Stage 1–9 Python modules, not in Blender.

## Files changed / added

- `src/aifr/fleet/registry.py`: adds **incremental robot registration** without resetting existing orders or robots.
- `tools/stage79_engine.py`: registers additional physically separated robots (V14 limit: four).
- `tools/live_pedestrians.py`: handles runtime-created walking agents, with map/path and spawn validation.
- `tools/stage79_dashboard.py`: existing dashboard now exposes live order, robot and pedestrian commands via a localhost HTTP service.
- `tools/blender_live_integration.py`: Blender sidebar controls and live visualization, polling Python snapshots every 0.2 seconds.
- `tests/test_live_runtime.py`: tests runtime changes and safety rejections.

## Installation: run inside your existing `.git` repository

```bash
cd ~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System
python3.12 -m venv .venv  # only if you do not have an existing working .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
python -m pytest -q tests/test_live_runtime.py
```

Do **not** re-run the virtual environment creation command if a working `.venv` already exists.

## Terminal 1: launch the live Python engine

```bash
cd ~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System
source .venv/bin/activate
python tools/stage79_dashboard.py --robots 1 --port 8765
```

Keep this terminal running. The engine starts **paused**, with one initial robot,
zero orders and zero pedestrians. Open http://127.0.0.1:8765 to interact from a
browser as well. The engine supports up to four robots in this V14 implementation.

## Terminal 2: launch the live Blender interface

```bash
cd ~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" \
  --python "$(pwd)/tools/blender_live_integration.py"
```

In Blender select the **Layout** workspace, hover over the 3D viewport, press
**N**, select the **AIFR** tab. Use **Start** to advance simulation time.

### Live controls

1. **Add Robot** creates an available robot with its own validated base. It preserves existing assignments. Up to four robots total; later admissions are blocked.
2. **New READY order**: enter a valid table (such as `T11`) and food station (such as `SANDWICH`) and click Submit. Orders enter READY-time FCFS. Robots stay BUSY until back at base.
3. **Spawn pedestrian**: enter *start X/Y* and *end X/Y* (metres), or click *Use 3D cursor position* for a stationary pedestrian. Moving pedestrians patrol between the two points. Invalid or occupied paths are rejected; multiple pedestrians can be added during a run.
4. **Pause / Step / Start**: perform simulation control while Blender keeps polling state. Emergency Stop latches until Reset via the browser dashboard.
5. Browser dashboard offers the same controls and more detailed event monitoring.

No values in a scenario configuration file have to be edited to create orders or
pedestrians. Events are submitted while the simulator runs, and the Python
engine decides whether they are physically allowed.

### Important limitations

- This is a **live Blender viewport connected to the Python simulator**, not
  physics or obstacle avoidance computed inside Blender. The same A*/ALNS/DWA,
  scheduling, LiDAR and EKF modules used for offline tests remain authoritative.
- The service runs locally on `127.0.0.1:8765`. Blender must be launched while
  the service is running. Port 8765 must be free. Blender's script must be launched
  with `--python`; merely opening the existing `.blend` won't install the sidebar.
- The recovered cafeteria `.blend` is never saved/overwritten by the integration.
- A fleet of **up to four robots** is presently supported; the Stage 2 registry
  requires at least one initial robot. Limit for pedestrians: **12**.
- Pedestrian motion follows user-defined smooth back-and-forth line segments.
  This is a kinematic model, **not an autonomous human-crowd simulator**.
- LiDAR tracking handles selected synthetic pedestrian scenarios; **arbitrary
  congested multi-robot/multi-pedestrian layouts have not been verified safe**.
  The collision guards can stop the simulation when unsafe, instead of making
  a maneuver magically succeed. Test incrementally from a simple scenario.
- Collision-inflated occupancy grid can reject some visually open pedestrian
  walkways. This is intentional; never bypass it just to force a spawn.
- The Blender scene only shows latest positions; it does **not** persist a
  keyframed trace in the source file. The older offline exporter still works.

## Regression and Git

```bash
python -m pytest -q tests/test_live_runtime.py
python -m pytest -q

git status
# Review modifications before committing
git add src/aifr/fleet/registry.py tools/stage79_engine.py \
  tools/stage79_dashboard.py tools/live_pedestrians.py \
  tools/blender_live_integration.py tests/test_live_runtime.py \
  LIVE_BLENDER_INTEGRATION.md
git diff --cached --stat
git commit -m "Add runtime commands and live Blender visualization"
```

Do not commit `.venv/`, generated `outputs/`, or scene backups. The new integration
**does not require renaming** `tools/run_stage79.py` or the old offline exporter.
