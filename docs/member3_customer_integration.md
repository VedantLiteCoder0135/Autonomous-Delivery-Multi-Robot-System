# AIFR Member 3 — Customer seating, service and departure integration

## Purpose / ownership

This package implements Member 3's features 7–9 for the **existing** V14 Blender-native
multi-robot cafeteria controller. It does **not** rewrite A*, ALNS, DWA, EKF,
FCFS scheduling, robot return-to-base or deadlock handling.

**Owned by Member 3:** `tools/customer_seating.py`, `tools/customer_lifecycle.py`,
`tools/customer_pose_adapter.py`, `tools/customer_delivery_adapter.py`,
`tests/test_member3_customer_behaviour.py` and
`data/member3_customer_seats_reference.json`.

**Shared integration (Member 1 review required before merging):**
`tools/customer_flow.py`, `tools/blender_integration_live.py`,
and one small phase-list change in `tools/stage79_engine.py`.
The original `run_mac.command` in the user ZIP is preserved without modification.
Do not blindly push the integrated ZIP into a concurrently modified teammate clone.
Member 2 can consume `CustomerFlow.get_customer_observations()`.

## Final product behavior

- On arrival, choose a random **eligible table first**, then a random free chair
  at the table. Exactly four chairs per table. Capacity is the *actual* scene
  chair count, currently 18 × 4 = 72. No fifth reservation at any table.
- Chair reservations are atomic/idempotent. Every chair stores table ID, actual
  scene-derived centre/heading, reachable free-grid approach point, existing
  robot delivery waypoint, status and occupant. When full, a bounded number of
  additional arrivals wait in the existing (non-physical) `QUEUED` state; this
  is NOT a new animated or complex crowd queue.
- Preserve existing entrances → cash payment → order PREPARING flow. Navigate
  from CASH to the **chosen chair approach**, then interpolate for 0.6
  simulation seconds into the actual chair position. Mark the order READY only
  once seated, retaining the scheduler's authority.
- Blender-native mode draws a small articulated customer avatar with a sitting
  pose (rather than a floor disc at an arbitrary aisle anchor). It uses chair
  orientation and the seat height from the recovered scene's construction
  script. No saved `.blend` is modified.
- Current walking/sitting/standing customer positions feed the existing DWA
  collision provider. A fully seated customer's body is covered by the
  pre-existing inflated static chair/table footprint: it is *not duplicated*
  as a second dynamic circle on the same furniture. Seated people's actual
  positions remain available to Member 2 through
  `get_customer_observations()`. This avoids falsely blocking valid robot
  delivery waypoints while preserving the static furniture collision guard.
  Free aisle floor is not automatically treated as a new robot path.
- On authoritative `DELIVERED_RETURN_PENDING` after `OrderStatus.DELIVERED`,
  a bridge validates matching order/customer. This simulation does not model
  a separate waiter; **the physical robot hand-off is also the serving event**.
  Only the `on_food_served` hook starts a `Uniform(20,40)` real-second timer.
  The `on_food_delivered` hook alone never starts it.
- Monotonic real time is used even at Blender 2×/4×/8×/16× speed.
  Eating time continues while paused, and the transition occurs on the next
  active simulation tick (documented default). No threads, sleeps or blocking
  callbacks. Duplicate events do not resample/reset.
- At deadline, stand for 0.6 simulation seconds, return to an accessible free
  grid approach point, reserve a reachable existing doorway, release the chair
  **after** clearing it, walk to exit and remove the customer. The chair becomes
  available for reuse immediately after clearing.
- Cancellations/removals release reservations. In-flight orders already held
  by a robot cannot be casually removed, preserving robot/order custody.

## File-level integration

`tools/customer_flow.py` now provides:

```python
flow = CustomerFlow(
    engine,
    customer_seating_enabled=True,  # legacy tests default to False
    seed=23,                        # testable random allocation
    now_real_s=time.monotonic,
)
customer_id = flow.add(entrance="main_entry", table="AUTO", station="SANDWICH")
flow.step(.05, now_real_s=time.monotonic())
flow.get_customer_observations()   # stable id, actual x/y, vx/vy, radius, timestamp
flow.on_food_delivered(order_id, customer_id, now_real_s=...)
flow.on_food_served(order_id, customer_id, now_real_s=...)
flow.remove_customer(customer_id, reason=...)
```

`tools/customer_delivery_adapter.py` consumes the **existing** engine event
`DELIVERED_RETURN_PENDING`, generated only after order status is set DELIVERED,
and calls the two customer hooks with the same monotonic time. It ignores
unlinked/manual orders. No scheduler code changes.

`tools/blender_integration_live.py` makes new behavior the default in the
Blender AIFR sidebar. **Reset** to apply a change to the `Chair seating` flag.
Customer add uses random table mode. Manual order table selection remains
separate. The Blender pose adapter supplies pooled parts and removes departed
customers; the controller also displays chair occupancy and remaining seconds.

`tools/stage79_engine.py` adds `TO_EXIT` to the existing set of pedestrian
routes considered when finding safe robot retreat positions.

## Scene geometry audit and limitations

`data/reference_cafeteria/stage3_environment.json` provides 18 real table IDs
and 72 chair ID/rect entries. The chair-centre selection and accessible approach
are computed at runtime from it and the existing inflated occupancy grid.
All 72 exported seats are mappable with valid approach cells. The compressed
`AIFR_Cafeteria_V14_Recovered.blend` binary also contains all 72 named chair
SEAT objects (checked by decompressing binary contents). A precise visual
transform/rig audit, animation appearance and clipping check remain **UNVERIFIED
without running Blender**.

No Blender application is installed in the current coding environment. Blender
viewport rendering, frame-by-frame rig posture and multi-robot visual passage
must be checked locally. The simplified articulated avatar does not reuse a
character rig; if a supplied rig is preferred, replace only the pose adapter.
No new real-world clearance guarantee is implied.

## Run in Blender (macOS, standard installation)

```bash
cd /path/to/Autonomous-Delivery-Multi-Robot-System
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" \
  --python "$(pwd)/tools/blender_integration_live.py"
```

In the 3D viewport press `N` → `AIFR`, ensure `Chair seating` is enabled
and click **Reset**. Start the simulation, add 2–4 customers from the panel,
observe assigned chair IDs, pose/animations, completed robot delivery, real
timer, exit, seat release and new customer reuse. Try speeds 1× and 16×.
Check visually that seated and moving customer bodies do not clip through
tables/chairs, that no robot crosses occupied furniture and that movement
continues without freezes. Preserve a backup of the `.blend` for the demo.
The controller only adds temporary runtime avatar objects.

## Tests / evidence

```bash
python -m pytest -q tests/test_member3_customer_behaviour.py tests/test_customer_flow.py
python -m pytest -q
```

Tests cover 72/18/4 geometry, random seeded table/chair selection, no duplicate
reservations, all-full fallback, existing queue FIFO/legacy paths, real chair
position, order READY gating, correct idempotent order/serving events, real-clock
independence, standing/departure, seat reuse, cancellation, floor approach
geometry, stable current observations and **a real Stage79Engine robot
delivery** with the customer waiting at T11.

See `MEMBER3_TEST_RESULTS.md` for exact test counts and limitations.

## Review/push discipline

1. Compare the changed paths and both supplied patches against your latest `main`.
2. Let Member 1 apply the **shared integration** patch and Member 2 review
   customer-observation compatibility. New M3-only files can be staged
   independently.
3. Do not overwrite the `.blend`, `run_mac.command`, robot scheduler or
   other teammates' work. Do not use `git add .`; stage reviewed paths only.
4. Rebase on updated main when safe, run focused + full tests again.
5. Commit and push only after the team's coordinated review; no pushes were
   made by this delivery.
