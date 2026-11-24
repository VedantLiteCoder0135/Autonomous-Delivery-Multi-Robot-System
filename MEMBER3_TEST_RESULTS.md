# AIFR Member 3 — Verification and delivery notes

Source ZIP: `Autonomous-Delivery-Multi-Robot-System.zip` (user-uploaded).
Repository commit recorded in the ZIP: `5253be1423f0a389e5e350565ad8a18dd561e05d`.
Existing uncommitted modification to `run_mac.command` was preserved unmodified.

## Verified

| Check | Outcome |
|---|---|
| Exported scene geometry | 18 tables, 72 named chair geometries, four chairs per table |
| Binary Blender asset | 72/72 expected chair `*_SEAT` names found in decompressed original `.blend` bytes |
| Chair approach mapping | All 72 chairs mapped to reachable free grid cells |
| Physical movement approach | Paths to actual chair approaches; no sampled transition segment crosses another chair/table |
| Random seat allocation | Uniform random eligible table, then free chair; seed reproducible |
| Seat reservation / release | No double booking; occupied status and chair reuse verified |
| Correct serving / timer | Delivery ID and OrderStatus verified; serving samples 20–40 seconds once |
| Speed / paused timing policy | Fake monotonic clock tests show simulation speed does not alter deadlines |
| Customer departure | Stand, clear chair, release, route to existing exit, remove from active customers; log actual real elapsed duration |
| Cancellation / cleanup | Release on cancel and external terminal order cancellation verified |
| Human observation adapter | Stable IDs, measured positions and timestamps; seated position at chair |
| Duplicate static/dynamic avoidance | Seated avatars covered by preexisting inflated chair/table geometry; no redundant DWA person circle |
| End-to-end engine test | Single customer, real Stage79Engine order delivered/served and timer linked |
| Multi-agent stress test | 2 robots, 3 customers (two share T11); all three served, exited, all seat counts returned zero, no engine failures |
| Legacy feature-off | Existing customer flow still works; nine original customer tests pass |
| Patch integrity | Member 3 and shared patches apply cleanly to the source commit |

## Test commands

```bash
# Focused tests include new slow 2-robot/3-customer integration.
PYTHONPATH=.:src python -m pytest -q tests/test_member3_customer_behaviour.py tests/test_customer_flow.py

# Normal full regression, excluding existing/marked slow stress tests:
PYTHONPATH=.:src python -m pytest -q -m "not slow"

# Explicit multi-robot stress test:
PYTHONPATH=.:src python -m pytest -q tests/test_member3_customer_behaviour.py::test_two_robots_three_customers_all_served_and_exit
```

Observed focused run before adding two final stress/occupancy cases: `21 passed in 7.57s`.
Four-customer physical seating test: `1 passed in 0.51s`.
Explicit multi-robot stress test: `1 passed in 16.16s`.
**Final focused suite (new + old customer tests): `23 passed in 23.55s` (exit code 0).**
Earlier non-slow regression run (before latest static-seat refinement): `759 passed, 8 deselected in 69.45s`.

**Final non-slow regression result:** `759 passed, 9 deselected in 69.29s` (exit code 0).
The nine deselected tests include the new dedicated multi-robot stress test,
which was run separately and passed, plus eight pre-existing slow tests.

An attempt to run the *full* unfiltered suite (including older slow stress tests)
was stopped without a final result due to its much longer runtime. No claim of
all slow tests passing is made.

## Unverified

Blender is not installed in this execution environment. The following require
running the supplied project in the user's Blender environment:

- Actual 3D customer pose appearance and skin/object geometry/clipping.
- Scene chair transforms beyond referenced exported geometry and verified names.
- Visual customer sitting/standing and 1× vs 16× wall-clock demo.
- Live multi-robot maneuvering past chairs in the Blender viewport.
- Confirm compatibility with teammates' latest concurrent `main` commits before push.

The baseline `.blend` file was **not** edited. Blender content has no generated
viewport screenshot or video; no viewport success is claimed.

## Integration notes

The complete updated project ZIP includes both owned and shared working code.
Two accompanying `.patch` files separate:
1. Member 3's exclusively owned new files and tests.
2. Shared controller/customer flow/engine wiring, to be reviewed and merged by Member 1.

Review and apply these before committing. No commit or GitHub push was made.
