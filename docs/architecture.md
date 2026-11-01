# System Architecture

*Autonomous Canteen Delivery Robots · Technical design reference*

[← README](../README.md) · [Environment](environment.md) · [Development plan](phases.md)

## 1. Design overview

The system is a **Blender-based, multi-robot food delivery simulation** with a **Python robotics core**. Its key architectural choice is to keep order management, navigation, localization, perception, robot coordination and visualization separate.

This document defines **responsibilities and interfaces**. It does not replace the task-by-task implementation plan in [phases.md](phases.md), and descriptions of planned behavior are not claims of completed functionality.

### System-level data flow (target)

```text
Customer / staff actions
         |
         v
Order manager ---> READY order queue
                        |
                        v
                 FCFS fleet scheduler <---- Robot registry / availability
                        |
                        v
                  Assigned mission
                        |
         +--------------+--------------+
         |                             |
         v                             v
 Cafeteria map + waypoints       Robot pose estimate
         |                             ^
         v                             |
  Global route / task plan          Odometry + sensors
         |                             |
         +--------------+--------------+
                        |
                        v
             Local motion controller
                        |
                        v
             Differential-drive model
                        |
                        v
             Simulated robot / Blender
                        |
                        v
          State updates, events, metrics
```

The scheduler operates at **fleet level**. Navigation and state estimation operate **per robot**, with the fleet coordination layer resolving shared-space conflicts.

## 2. Module boundaries

The reference package layout from the development plan is `src/aifr/`. Directories may be introduced incrementally.

| Package | Responsibility | Typical inputs / outputs |
|---|---|---|
| `common/` | Shared models, events, errors and clock | Types and events used across modules |
| `config/` | Central simulation parameters | Robot dimensions, timing, limits |
| `orders/` | Order states, preparation, completion, cancellation | Order events and state changes |
| `fleet/` | Robot identity, state and availability | Registry entries and robot events |
| `scheduling/` | Global READY queue and deterministic assignment | READY orders + eligible robots → assignments |
| `environment/` | Blender scene extraction, classification and waypoints | Scene metadata → geometry / candidate points |
| `navigation/` | Occupancy representation, collision checks, planning and control | Map + pose + goal → route / motion commands |
| `perception/` | Simulated sensors and localization | Sensor/odometry data → observations and pose estimate |
| `coordination/` | Shared-corridor, reservation, yielding and deadlock logic | Robot intentions → conflict-resolution actions |
| `simulation/` | Time stepping and orchestration | Events + robot updates → simulation state |
| `interface/` | Blender/UI adapters, controls and display | Simulation state ↔ visualization/user commands |

**Boundary rule:** `bpy`-dependent extraction and rendering stay in Blender-facing adapters. The order, scheduling, geometry and planning logic should remain testable with ordinary Python wherever possible.

## 3. Orders and dispatch

### Order lifecycle

```text
CREATED -> PREPARING -> READY -> ASSIGNED -> TO_PICKUP
        -> LOADING -> TO_CUSTOMER -> UNLOADING -> DELIVERED

Exceptional terminal states: FAILED, CANCELLED
```

An order specifies a **food counter** and a **destination table**. A table should resolve to a safe approach waypoint, not the table centre. Staff mark food **READY** before it becomes dispatchable; robot loading and delivery confirmations are simulated events.

### Currently documented scheduler policy

The completed scheduling module described in `phases.md` follows these rules:

- **One global FCFS queue** across the fleet, populated when orders become **READY**, not at initial creation.
- Earlier **READY timestamps** have higher priority; a sequence number resolves ties.
- Available robots are chosen deterministically by **ascending robot ID** unless explicit robot priorities are configured. Robot priority does not reorder waiting orders.
- **One active order per robot**. Busy or faulted robots cannot accept another order.
- No distance-based assignment or route-cost-based scheduling is part of the current FCFS policy.
- A faulty robot becomes `FAULTED`; its active order becomes `FAILED` and is **not requeued automatically**. Manual retry creates another order.
- Cancellation is rejected once the order reaches `LOADING`.

This policy must remain the baseline when later optimization work is introduced. Any change to fairness, eligibility, batching or tie-breaking needs an explicit design decision and corresponding tests.

### Robot lifecycle and availability

The **intended end-to-end physical workflow** is:

```text
IDLE at base -> travel to counter -> load -> travel to table
             -> unload -> return to base -> IDLE
```

The **scheduler currently documented in the development plan** makes a robot available immediately after delivery, without performing the return-to-base trip. The return-to-base requirement is not yet integrated with the fleet state machine. Until then, do **not** claim that only base-camp robots are eligible or that return motion is implemented.

The eventual implementation needs a decision about when the robot becomes `IDLE`, how the base camp is represented, and how interrupted/blocked returns affect scheduling.

## 4. Navigation, perception and optimization

| Layer | Intended method | Owns | Does not own |
|---|---|---|---|
| Map and traversability | Geometry + occupancy grid / collision checking | Free/blocked space and robot clearances | Order priority |
| Global route / task optimization | **ALNS** (planned) | Solve an explicitly defined routing problem using valid map constraints | Wheel commands or local avoidance |
| Optional pathfinding baseline | **A\*** (earlier prototype) | Reference point-to-point path, if retained | Fleet assignment |
| Local motion | **DWA** (planned) | Feasible linear/angular velocity commands | Global queue policy |
| Localization | **EKF** (planned integration) | Estimated pose and uncertainty | Collision-free path guarantees |
| Perception | **Simulated 2D LiDAR** / observations | Nearby environment measurements | Task dispatch |
| Fleet traffic | Reservations, yielding, conflict/deadlock handling (planned) | Safe interaction of multiple robots | Individual sensor fusion |

**Do not treat A\* and ALNS as interchangeable.** The team intends to explore ALNS for global optimization, while A\* belongs to an earlier point-to-point prototype and is **optional**, not a committed part of the final architecture. ALNS needs a defined optimization problem, route representation and path-feasibility checks. The currently documented **one-order-per-robot FCFS** policy does not by itself define a multi-stop ALNS problem. A working route-generation strategy is also needed for single-robot delivery before any fleet optimization is integrated.

### Intended per-robot navigation cycle

1. Read odometry and simulated obstacle/landmark observations.
2. Estimate pose with EKF, keeping ground truth separate for evaluation.
3. Use the approved map and a validated pickup/delivery waypoint.
4. Obtain or update global path guidance when required.
5. Choose safe local motion using DWA and collision constraints.
6. Apply differential-drive commands; advance the simulation and record state.

The nominal design uses a **0.05 s (20 Hz) simulation step**, configurable. It is a target setting, not verified real-time performance. A successful planner must also validate the whole oriented robot footprint, not just the robot centre.

### Multi-robot coordination

Robot-to-robot collisions cannot be prevented by independent route planning alone, especially in narrow aisles. The fleet design therefore anticipates conflict detection, passage reservations or equivalent coordination, yielding, waiting and deadlock handling. These are **planned** capabilities until demonstrated in the integrated simulation.

## 5. Environment and data exchange

The approved Blender cafeteria scene is the geometry source. The `environment/` layer extracts and classifies it, then provides **Blender-independent data** for mapping and navigation.

Data-exchange requirements:

- Record world-space transforms and dimensions, including relevant parent/scale effects.
- State **coordinate conventions, units (metres)**, data/schema version and object identity.
- Preserve **unknown or unsupported objects** as explicit diagnostics, not silent free space.
- Expose walkable floor regions, obstacles, entrances and waypoint candidates.
- Validate rotated robot footprint clearances and waypoint accessibility.
- Use **JSON** as the intended interchange format between scene extraction and pure-Python components.

Specific scene dimensions, counts, constraints and validation rules belong in [environment.md](environment.md).

## 6. Testing and correctness

| Area | What should be verified |
|---|---|
| Order management | Valid state transitions, READY queue order, cancellation and fault rules |
| Fleet scheduling | FCFS fairness, tie-breaks, busy/faulted robot exclusion, repeatable results |
| Geometry | Transform extraction, L-shaped floor union, obstacle classification |
| Traversability | Oriented footprint checks, narrow passages, door gaps, corner cutting |
| Navigation | Valid endpoints, obstacle-free paths, route following, safe stopping |
| Localization | Estimation error against ground truth, behavior with noisy data |
| Coordination | No robot collisions or deadlocks in defined stress scenarios |
| Integration | Order → pickup → delivery behavior, state consistency and useful metrics |

The development plan reports **511 passing scheduling-related tests and high coverage** at handoff. Those are **reported historical results**, not proof that Blender navigation or the full multi-robot workflow is currently integrated. Any displayed test count should be tied to a verified run and revision.

## 7. Open integration choices

These questions must be resolved in the implementation process rather than silently answered by documentation:

1. **Return to base:** target behavior versus immediate post-delivery availability in the current scheduler.
2. **Global routing:** define how collision-free point-to-point routes will be generated for the first delivery demo; decide whether A\* is retained only as a comparison baseline.
3. **ALNS scope:** define objective, constraints and whether multi-order batching is allowed without breaking FCFS.
4. **Repository shape:** use the real `src/aifr/` package, not a second competing legacy `backend/` tree.
5. **Simulation launch:** document a full run command only after the Blender and Python components are integrated and verified.

For implementation sequencing and completion checks, consult [phases.md](phases.md). This file is the architectural reference, not a second roadmap.
