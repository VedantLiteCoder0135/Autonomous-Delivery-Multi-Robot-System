# AIFR integrated hardening: verification and limitations

This package is meant to be copied into the **existing Git checkout**. It does
not contain a `.git` directory and does not modify the recovered V14 Blender
scene. Back up or commit local changes before overwriting same-named files.

## What changed from the previous live Blender patch

1. **Order ownership:** `create_order(..., customer_id=...)` records a two-way
   customer/order association; repeat payment for one customer is rejected.
   Robot pickup/drop-off checks the assigned robot and customer-order custody.
   The Blender sidebar now exposes order/customer/robot IDs.
2. **Customer traffic:** Single-slot cashier admission is global FIFO across
   entrances. Walking customer sweeps check both robot and pedestrian bodies.
   Bounded rerouting overlays nearby people and robots on a temporary grid.
   Four customers per table and door waiting queues remain supported.
3. **Narrow geometry:** `tools/geometry_hardening.py` checks pickup egress
   and whether candidate waiting robots sever map reachability to food
   sections. It uses the recovered V14 scene and leaves it unchanged.
4. **Recovery:** Stationary robots that exceed a DWA/traffic wait threshold
   attempt at most two route replans, preserving the order ID. If recovery
   remains impossible, a visible FAULT is reported. No unchecked reversing.
5. **Live controls:** Direct Blender control panel with manual orders,
   customer entry/payment/automatic orders, live robot addition and speed
   targets 1x–16x. A wall-clock work cap protects the Blender UI; requested
   playback speed may be slower than real-time on CPU-bound machines.
6. **Regression:** New fast deterministic tests and three end-to-end scenario
   runners for repeat orders, left-side shops, and table capacity.

## Install inside the GitHub-linked repository

```bash
cd ~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System
git status
# Commit or back up local modifications before copying the package.
# The complete ZIP is rooted at project root; unzip INSIDE the repository:
unzip -o ~/Downloads/AIFR_Integrated_Hardening_Final.zip -d .
python3.12 -m venv .venv  # only if .venv doesn't exist
source .venv/bin/activate
python -m pip install -e '.[dev]'
python -m pytest -q
python -m tools.run_geometry_audit
python -m tools.run_geometry_hardening
python -m tools.verify_hardening --scenario identical
python -m tools.verify_hardening --scenario left_side
python -m tools.verify_hardening --scenario four_customers
```

Start the Blender-native UI (no server):

```bash
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" \
  --python "$(pwd)/tools/blender_integration_live.py"
```

Inside Blender: **N → AIFR**. Start, add customers, add robots, or add
manual orders; observe unique identities and traffic warnings. Test 1x, 2x,
4x, 8x and 16x without modifying physics dt.

## Verification in authoring environment

- 694 regression tests passed, excluding `test_stage79_system.py` and the
  separately run new hardening tests.
- 7 lightweight system tests passed; 2 long-running system tests excluded
  from that subset because the full suite exceeded the execution time limit.
- 9 focused hardening tests passed; 29 focused hardening, customer, traffic
  and geometry tests passed together.
- Direct end-to-end **three customers / two identical Sandwich orders + Tea**:
  three distinct orders delivered, both robots returned, no reported faults.
- Direct end-to-end **Sandwich + MAIN_COURSE to T11**: both delivered,
  returned, no reported faults.
- V14 geometry analysis: **7/7 pickup egresses available** and **5/5**
  waiting candidates preserved static pickup connectivity.

These are sampled simulations, not guarantees for every live crowd configuration.

## Explicitly NOT guaranteed

- Full deadlock freedom in arbitrary crowded environments. The Blender live
  runtime uses an **exclusive shared-aisle robot permit**, which avoids
  opposing active robot traffic but sacrifices robot concurrency.
- Reverse maneuvering and arbitrary narrow corridor recovery are not yet
  certified. A physically impossible route produces a fault rather than a
  fictitious completion.
- Pedestrians are circles moved kinematically, with conservative collision
  checks; they do not have a full crowd model or physical seated meshes.
- Geometry audit validates static A* egress and stationary turns, not proof
  of local DWA motion feasibility under all traffic conditions.
- Blender GUI was not executed in this environment. Run the Blender smoke
  test on your Mac before committing results as fully verified.
- The full pytest suite timed out here near 95% completion. Re-run it locally.

## Git workflow

```bash
git diff --stat
git status
# Never commit .venv, __pycache__, generated outputs, or temporary .blend1 files.
git add README.md docs tools tests src INTEGRATION_VERIFICATION.md
# Inspect staged changes before committing.
git diff --cached --stat
git commit -m "Harden AIFR live orders and narrow-aisle traffic"
```

If your main branch is protected, push the change to a feature branch and
open a pull request instead of committing directly to main.
