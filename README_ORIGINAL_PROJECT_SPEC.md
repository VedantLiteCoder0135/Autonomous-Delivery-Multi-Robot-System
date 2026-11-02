# Autonomous Canteen Delivery Robots

A simulation of **3-4 delivery robots** serving food in a real-life, L-shaped canteen. The 3D canteen is modelled in **Blender**, and the robots are simulated in **pure Python** (no ROS).

Built for the *AI for Robotics* course at BITS Pilani, Hyderabad Campus.

---

## What it does

1. A customer places an order. The order goes into a **first-come-first-served (FCFS) queue**.
2. When one or more robots are free at the **base camp**, the **highest-priority robot** takes the next order.
3. The robot drives from the pickup point (e.g. a food counter) to the customer's table. Each order has its own start and goal.
4. On the way it avoids walls, tables, **moving people** and the other robots.
5. After delivering, it returns to the base camp and waits for the next order.

## How the robot works

| Part | Algorithm | What it does |
|---|---|---|
| Global path | **A\*** | Finds an obstacle-free route on the canteen map |
| Route planning | **ALNS** | Optimises the order of stops when a robot has several deliveries |
| Local avoidance | **DWA** | Steers around people and other robots in real time |
| Localization | **EKF** | Estimates where the robot is from noisy odometry and landmark readings |
| Order handling | **FCFS + priority** | Decides which order is served first and which robot takes it |

The robots are **differential-drive** (two powered wheels), with a simulated 2D LiDAR.

## Project status

This project is under active development.

| Feature | Status |
|---|---|
| Differential-drive model, LiDAR, EKF | Working (single robot) |
| A* planner and pure-pursuit controller | Working |
| Dynamic obstacles (people) | Working |
| 3D simulation | Working |
| Canteen map (from the Blender layout) | In progress |
| Multiple robots and base camp | In progress |
| FCFS queue and priority scheduling | Planned |
| DWA local planner | Planned |
| ALNS route optimisation | Planned |
| Multiple orders per robot | Planned |
| Cost optimisation of each robot (distance, time, energy) | Planned |
| Battery model and charging at the base camp | Planned |
| Robot-to-robot traffic rules (yielding at narrow spots, deadlock avoidance) | Planned |
| Order deadlines and urgent orders | Planned |
| Rush-hour scenario with a crowded canteen | Planned |

## Project structure

```
autonomous-multi-robot-system/
├── main.py                  # run this to start the simulation
├── backend/                    # simulation code
│   ├── config.py            # all settings in one place
│   ├── canteen_map.py       # canteen layout and base camp
│   ├── dynamic_obstacles.py # walking people
│   ├── robot.py             # one robot (sensors + EKF + planner + controller)
│   ├── kinematics.py        # differential-drive motion
│   ├── sensors.py           # LiDAR and landmark sensor
│   ├── ekf.py               # localization
│   ├── costmap.py           # obstacle map with safety margins
│   ├── path_planner.py      # A*
│   ├── dwa.py               # local obstacle avoidance
│   ├── alns.py              # route optimisation
│   ├── controller.py        # path following and safety stop
│   ├── cargo.py             # orders
│   └── scheduler.py         # FCFS queue and priority dispatch
├── blender/
│   └── build_canteen.py     # builds the 3D canteen in Blender
├── tests/                   # unit tests
└── docs/                    # slides and report
```

## How the system works

Every robot runs the same loop, 20 times per second:

```
 Sensors (LiDAR, odometry, landmarks)
        |
        v
 EKF localization  -->  "where am I?"
        |
        v
 A* global path  -->  "which route to the goal?"
        |
        v
 DWA local planner  -->  "how do I avoid the person in front of me?"
        |
        v
 Controller  -->  left and right wheel speeds
```

The scheduler sits above the robots and decides who does what:

```
 New order --> FCFS queue --> Scheduler --> free robot with highest priority
```

## Robot life cycle

Each robot is always in one of these states:

```
 IDLE at base camp --> GOING TO PICKUP --> GOING TO TABLE --> RETURNING --> IDLE
```

- **IDLE:** waits at the base camp. Only idle robots can receive orders.
- **GOING TO PICKUP / TABLE:** follows the A* path and avoids obstacles with DWA.
- **RETURNING:** drives back to the base camp after the delivery.

## Scheduling rules

- Orders are served **first come, first served**.
- If several robots are idle, the one with the **highest priority** takes the order.
- Each order has its own **start** (e.g. a food counter) and **goal** (a table), so routes differ from order to order.
- ALNS is used when a robot has more than one stop to visit, to pick the best order of stops.

## How we measure results

| Metric | Why it matters |
|---|---|
| Collisions | Must be zero |
| Localization error (EKF vs. true position) | Shows how accurate the position estimate is |
| Path length vs. shortest path | Shows how efficient the routes are |
| Replanning time | Must be fast enough for real-time use |
| Order waiting time | Shows how well the queue and scheduler work |
| Delivery time and robot utilisation | Shows how well the fleet is used |

## Team

- `VEDANT SHAH`
- `JENIL KARIA`
- `VEDANTH GADA`

Course: AI for Robotics, BITS Pilani, Hyderabad Campus.