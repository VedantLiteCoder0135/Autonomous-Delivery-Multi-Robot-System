# AIFR: Blender-native live customer workflow

This is an additive integration patch for the **existing Git repository**, not a new project. It retains the tested Stages 1–9 core, including A*, DWA, EKF, ALNS route geometry, FCFS dispatch, return-to-base and the four wall-verified entrances. It does **not** use the localhost server or the old `blender_live_integration.py` HTTP client.

## Install

Run in your existing `~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System` checkout:

```bash
# Check whether you have source changes to preserve before extracting
 git status
# Extract the accompanying ZIP into the repository root
 unzip -o ~/Downloads/AIFR_Blender_Native_Customer_Flow.zip -d .
# Use an existing Python virtual environment if available
 source .venv/bin/activate
 python -m pytest -q tests/test_customer_flow.py
 python -m pytest -q
```

## Launch, without a Python server

In your project root:

```bash
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" \
  --python "$(pwd)/tools/blender_integration_live.py"
```

In Blender's **3D Viewport**, press **N**, open the **AIFR** tab.

1. **Start** begins the Python simulation loop, driven by a Blender timer.
2. Select an **entrance** (Automatic or one of four verified openings), a **table**, and a **food station**.
3. **Add Customer** creates the pedestrian at an interior point near the selected entrance. You no longer enter XY coordinates.
4. The customer walks along a grid-validated A* route to **CASH**. After a short simulated payment delay, the engine calls its existing `create_order(..., ready=False)` method. This is the automatic order submission.
5. The customer proceeds to a reachable anchor adjacent to the requested table, away from robot delivery waypoints. The customer becomes **SEATED**, the order becomes **READY**, and FCFS scheduling can assign an available robot.
6. The robot goes to the chosen food station, delivers to the chosen table and returns to its assigned base before becoming available again.
7. **Add Robot**, **Pause**, **Step**, **Reset**, and **Emergency Stop** are Blender controls. The status panel lists recent customers and robot phases.

Generated objects use the `AIFR_LIVE_CUSTOMER_SIM` collection. The existing cafeteria geometry is not edited, and the source `.blend` file is never saved automatically.

## Important technical boundaries

- **Blender-native** means the existing pure-Python engine is imported into Blender's embedded Python and ticked from its main-thread timer. There is no HTTP backend, browser dashboard or secondary service. This is **live navigation**, not offline animation playback.
- Entrances' metadata points are physically located at wall openings; the conservative robot-inflated grid marks the exact metadata centers occupied. The customer begins at the nearest *reachable interior free grid cell*. This is a visual approximation to crossing the threshold, **not a simulated door-opening/physics interaction**.
- The customer uses an abstract colored avatar and a nearby, free seat anchor. The avatar does not physically sit in a chair mesh. Walking uses A*, not a crowd motion / social-force model. At most 12 customers, each with a distinct requested table, and up to 4 robots.
- Payment is an automatic timed event. It is **not a payment processor or banking transaction**. The order is created at payment, but marked READY only after seating so the delivery isn't dispatched before the customer reaches the table.
- Robot motion safety remains guarded by the existing Python engine. It has not been validated in live Blender with dense crowds and simultaneous customer paths. New customer routes are checked for static traversability, but robust customer-to-customer traffic management and deadlock recovery are future work. A successful single-customer flow is not a guarantee for crowded scenes.
- The timer advances 0.05 seconds of simulation per ~0.10 seconds of wall time, by default. Actual runtime speed depends on Blender and Python performance; UI responsiveness matters more than realtime ratio here.
- The earlier HTTP client (`tools/blender_live_integration.py`) and dashboard (`tools/stage79_dashboard.py`) remain in the repository for backwards compatibility, but **do not start either** when using this Blender-native workflow. Only run the command shown above.

## Verification

Run `python -m pytest -q tests/test_customer_flow.py` and then the existing test suite. An end-to-end pure Python scenario with entrance `main_entry`, CASH payment, T11 seating and a SANDWICH order completed and returned the robot to base without faults. The native Blender UI still requires user-side execution on macOS.

Before committing, inspect `git status` and stage only the new files:

```bash
git add tools/customer_flow.py tools/blender_integration_live.py tests/test_customer_flow.py BLENDER_NATIVE_CUSTOMER_FLOW.md
git diff --cached --stat
```
