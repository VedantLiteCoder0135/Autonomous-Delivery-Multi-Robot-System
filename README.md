# Autonomous Canteen Delivery Robots

**A multi-robot autonomous food delivery simulation in a realistic cafeteria.**

*AI for Robotics (AIFR) · BITS Pilani, Hyderabad Campus*

This project explores how a small fleet of autonomous mobile robots can collect food from cafeteria counters and deliver it to customer tables. The environment is modelled in **Blender**, while scheduling, navigation and robot logic are developed in **Python**. The aim is a robotics simulation driven by algorithms, **not a pre-animated scene**.

The target demonstration uses **3–4 differential-drive robots** in an L-shaped cafeteria, with static furniture, narrow aisles and moving obstacles. A physical robot build is outside the current simulation scope.

**Documentation:** [System architecture](docs/architecture.md) · [Cafeteria environment](docs/environment.md) · [Development plan](docs/phases.md)

---

## What the system does

1. A customer places a food order for a counter and a destination table.
2. Staff prepare the order and mark it **READY**. Only then does it enter the **first-come, first-served (FCFS)** dispatch queue.
3. The scheduler assigns the oldest READY order to an eligible robot, using robot priority to break simultaneous availability ties.
4. The robot navigates to the food counter, waits for simulated loading, and travels to a **safe delivery point beside the table**.
5. The delivery is confirmed. The longer-term fleet workflow includes returning to base camp before accepting another job.
6. Navigation and coordination are intended to keep robots clear of furniture, people and each other.

> **Implementation distinction:** The currently documented scheduler makes a robot available immediately after delivery. Return-to-base eligibility is a **target behaviour**, not an implemented scheduler rule. See [architecture](docs/architecture.md#robot-lifecycle-and-availability).

## How it works

The system separates **order assignment**, **route planning**, **motion control** and **state estimation** so that each part can be tested independently.

| Component | Approach | Purpose |
|---|---|---|
| Order scheduling | **FCFS + robot priority** | Assign READY orders fairly and deterministically |
| Global route / task optimization | **ALNS** (planned) | Optimize a defined routing problem subject to map and fleet constraints |
| Reference pathfinding | **A\*** (earlier prototype / optional baseline) | Generate comparison point-to-point routes if retained |
| Local motion and avoidance | **DWA** (planned) | Select safe velocity commands around nearby obstacles |
| Localization | **EKF** (planned integration) | Estimate robot pose from noisy motion and sensor measurements |
| Perception | **Simulated 2D LiDAR** | Measure nearby obstacles in the virtual environment |
| Multi-robot traffic | **Conflict handling / yielding** (planned) | Prevent collisions and deadlocks in shared narrow aisles |

**ALNS is the intended optimization approach; A\* is not a required part of the final design.** The earlier prototype used A\* for point-to-point pathfinding, which can remain a baseline if useful. ALNS is a more general optimization method, so its route representation, objective and feasibility constraints must be defined before using it for navigation. It must not silently change the FCFS order policy.

### Navigation concept

```text
             Customer order marked READY
                         |
                    FCFS scheduler
                         |
                   Robot assignment
                         |
                         v
              Pickup / delivery goal
                         |
               Global route / task plan
                         |
Sensors + odometry -> EKF pose -> Local motion (DWA)
                                         |
                               Differential-drive robot
                                         |
                                 Updated simulation
```

This is the **intended integrated design**, not a claim that the entire loop already works. The proposed nominal simulation step is **50 ms (20 Hz)**.

## Cafeteria simulation

The approved Blender model represents an **L-shaped canteen** with:

- **18 tables** and **72 chairs**
- **6 pillars** and narrow walking passages
- **4 food counters:** Main Course, Tea, Sandwich and Juice
- Hand-wash and cash/billing counters
- **4 entrances/exits**, including a sliding entrance near table T11

Pickup and delivery points must be validated against the robot's **full footprint**, not just its centre. The dimensions in the model are provisional assumptions, not surveyed measurements of the real cafeteria.

See [docs/environment.md](docs/environment.md) for the scene source, geometry conventions, obstacles and waypoint rules.

## Technology

| Area | Technology |
|---|---|
| 3D environment and visualization | Blender 4.5.2, Blender Python API (`bpy`) |
| Robotics, scheduling and simulation core | Python |
| Environment data exchange | JSON |
| Automated tests | `pytest` |
| Middleware | No ROS 2 / Gazebo dependency in the current design |

Blender-specific scene extraction and visualization are kept separate from the Python logic wherever practical, allowing algorithms to be tested without starting Blender.

## Current development status

The project is **under active development**. The status below distinguishes the current modular implementation from earlier prototype demonstrations.

| Feature | Current state |
|---|---|
| 3D cafeteria model and initial architecture | Prepared / reviewed; approved Blender scene retained |
| Order lifecycle and READY-based FCFS queue | Implemented in the staged Python package; tests reported |
| Fleet registry and priority-based robot assignment | Implemented in the staged Python package; integration with movement pending |
| Blender scene extraction and environment mapping | In progress; validation against the actual scene outstanding |
| Pickup, delivery and safe waiting waypoints | In progress; geometry and full-footprint checks still needed |
| Differential-drive robot motion | Reported working in an earlier single-robot prototype; staged integration unverified |
| Simulated 2D LiDAR and EKF localization | Reported working in an earlier single-robot prototype; staged integration unverified |
| A* pathfinding and pure-pursuit tracking | Reported working in an earlier prototype; optional baseline, not the committed final global planner |
| Moving-person obstacles and 3D robot visualization | Reported in the earlier prototype; end-to-end staged integration unverified |
| Occupancy grid, clearance and footprint collision checks | Planned |
| Complete single-robot pickup-to-delivery workflow | Planned for integrated demonstration |
| DWA local obstacle avoidance | Planned |
| ALNS route / task optimization | Planned; objective, constraints and FCFS compatibility to be defined |
| 3–4 robot coordination, narrow-aisle yielding and deadlock handling | Planned |
| Return-to-base workflow | Target behaviour; current scheduler makes robots available after delivery |
| Blender monitoring and control dashboard | Planned |
| Battery simulation, charging and multi-order batching | Optional future scope, not part of the verified implementation |
| Urgent orders, deadlines and rush-hour stress scenarios | Optional future scope / future demonstration scenarios |

For detailed implementation tasks, dependencies and completion criteria, use **[docs/phases.md](docs/phases.md)**. This README deliberately does not duplicate the development stages.

## Repository organization

The documented modular package layout is:

```text
aifr/
├── src/aifr/
│   ├── common/           # shared types, events and utilities
│   ├── config/           # configuration models
│   ├── orders/           # order lifecycle
│   ├── fleet/            # robot registry and fleet state
│   ├── scheduling/       # READY queue and assignment logic
│   ├── environment/      # scene extraction, mapping, waypoints
│   ├── navigation/       # mapping / planners / controllers (incremental)
│   ├── perception/       # simulated sensors and state estimation
│   ├── coordination/     # robot-to-robot conflict handling
│   ├── simulation/       # simulation orchestration
│   └── interface/        # simulation controls / visualization adapters
├── tests/
├── tools/
├── docs/
│   ├── phases.md
│   ├── architecture.md
│   └── environment.md
├── pyproject.toml
└── README.md
```

Some folders are planned and may not exist yet. This is the **staged package design**, not a promise that every module is already implemented. See [architecture](docs/architecture.md#module-boundaries).

## Running and testing

The Python modules have been designed for independent testing. From the repository root, with the project's Python dependencies installed:

```bash
python -m pytest
```

**Full Blender simulation launch:** A single verified, integrated entry-point command has not yet been established in the supplied project documentation. It will be documented here after integration; the old `python main.py` command belongs to a different/legacy layout and should not be assumed to work with the staged package.

## Evaluation goals

We intend to assess the simulation using **collision count**, **localization error**, **route efficiency**, **planning/replanning time**, **order waiting time**, **delivery time** and **robot utilization**. These are evaluation targets, not published performance results.

## Team

**Vedant Shah · Jenil Karia · Vedanth Gada**  
*AI for Robotics · BITS Pilani, Hyderabad Campus*
