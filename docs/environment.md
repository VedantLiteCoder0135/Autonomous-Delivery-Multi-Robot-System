# Cafeteria Environment

*Autonomous Canteen Delivery Robots · Scene and mapping reference*

[← README](../README.md) · [System architecture](architecture.md) · [Development plan](phases.md)

## 1. Environment overview

The simulation uses a **realistic, L-shaped cafeteria** constructed in **Blender**. Furniture, service counters, entrances and narrow passages create a more meaningful navigation challenge than an empty test room.

The authoritative scene is the team's approved `.blend` file and the associated source script:

```text
AIFR_Cafeteria_Corner_Handwash_Cash_V14.py
```

**Preserve the approved V14 geometry.** Scene extraction, mapping and navigation should read the existing objects without silently moving, deleting or resizing them. If a scene change is needed, treat it as a separately reviewed model update.

This document describes the **model and validation requirements**; it does not claim that all scene-to-map extraction or waypoint checks are already complete.

## 2. Layout and scale

The floor is approximately an **L-shaped union** of two rectangular regions:

| Region | X range | Y range |
|---|---|---|
| Lower arm | 0–19 m | 0–7.5 m |
| Upper wing | 9–19 m | 7.5–17.5 m |

These ranges are **provisional modelling assumptions**, not survey measurements. Actual Blender object transforms and the approved scene take precedence over manually typed values.

The floor must be interpreted as **connected walkable regions**, not a single 19 × 17.5 m rectangle: cells outside the L are not valid traversal space. The shared boundary between arms must not be accidentally treated as a blocking wall if the scene connects them.

## 3. Scene inventory

| Object group | Documented count / details |
|---|---|
| Customer tables | 18 |
| Chairs | 72, approximately 4 per table |
| Pillars | 6 |
| Food service counters | 4: Main Course, Tea, Sandwich, Juice |
| Other counters | Hand Wash, Cash/Billing |
| Entrances/exits | 4: main entrance, two side exits, sliding entrance near T11 |
| Robot | Existing placeholder; nominal footprint approximately **0.40 × 0.36 m** |

These are the counts recorded in the supplied project documentation. Any discrepancy during extraction must be **reported**, rather than papered over by inventing missing geometry.

### Service and delivery destinations

- **Pickup:** a validated approach pose at the appropriate food counter, with space for simulated loading.
- **Delivery:** a reachable approach pose beside the requested table, **never at the centre of its furniture**.
- **Spawn / base camp:** an explicitly validated safe pose or area, not an arbitrary empty-looking spot.
- **Waiting / yielding:** optional poses that leave room for other robots to move through shared aisles.

Pickup and drop-off poses must include position and, where relevant, orientation. Names such as `T11` or `Juice` identify targets but do **not** establish safe coordinates on their own.

## 4. Geometry extraction

The `environment/` modules described in the development plan are intended to extract the approved scene into data usable without Blender.

For relevant objects, extract or derive:

1. Stable identity/name and object type.
2. World-space location and orientation.
3. Dimensions and transformed bounding geometry.
4. Parent-child relationships where they affect world transforms.
5. Environment classification and whether an object blocks robot motion.
6. Any unresolved classification or geometry issues.

**Important:** Blender local transforms may be misleading when objects are parented, scaled or rotated. Mapping should use the effective **world-space geometry**. Axis-aligned bounds are not a substitute for an oriented footprint collision test when a rotation matters.

### Classification

| Class | Treatment in navigation data |
|---|---|
| Walkable floor | Candidate traversable regions, after containment checks |
| Walls / pillars | Static obstacles |
| Tables / chairs | Static obstacles, unless explicitly modeled otherwise |
| Food / hand-wash / cash counters | Physical obstacles with separate approach waypoints |
| Entrances / openings | Preserve actual gaps and access constraints |
| Decorative objects | Inspect and classify; some may still block motion |
| Unknown objects | Flag for review; do not silently classify as free space |

The **four actual entrances** must remain represented correctly. No door or passage should be inferred as traversable merely from its name.

## 5. Navigation map and robot footprint

The extracted scene feeds an **occupancy or equivalent navigation representation**. A starting grid resolution of **0.10 m** was proposed in the technical plan; it is configurable and must be checked against narrow-passages and obstacle dimensions.

Expected distinctions include:

- **Free:** verified, walkable space with adequate clearance.
- **Blocked:** furniture, walls, pillars, counter bodies or non-walkable outside regions.
- **Unknown:** insufficient or unclassified geometry that requires inspection.

Robot safety checks must use the **full oriented 0.40 × 0.36 m rectangular footprint**, plus a configurable clearance margin. Merely verifying that the robot centre lies on a free grid cell is inadequate.

A candidate route or waypoint should be rejected if the footprint:

- Crosses an outside edge of the L-shaped walkable floor.
- Overlaps furniture, pillars, counters or walls.
- Cannot pass through a narrow aisle at the required orientation.
- Clips a corner during a turn, even if the route endpoints are free.
- Reaches a doorway or destination that is not actually accessible.

## 6. Waypoint generation and validation

The planned waypoint layer (`waypoint_manager.py`) should find and check:

| Type | Validation requirement |
|---|---|
| Robot spawn | Inside a walkable region, collision-free |
| Base camp | Safe parking / dispatch location with adequate maneuvering space |
| Food pickup | Beside the correct counter, physically reachable |
| Table delivery | Reachable approach candidate for each of the 18 tables |
| Waiting / yielding | Clear of obstacles and not blocking critical passages |

For every candidate, report either **valid** or **rejected with a reason**. Validating a point in isolation is insufficient; connectivity and approach feasibility matter. Unreachable tables or counters must be reported explicitly.

The base-camp **location** is an environment-design concern. Whether robots must return there **before becoming available** is a separate scheduler/integration decision described in [architecture.md](architecture.md#robot-lifecycle-and-availability).

## 7. Output to the Python simulation

The scene extraction/mapping layer is intended to produce a **versioned JSON representation** (exact schema to be finalized). At minimum, downstream modules need:

- Coordinate system, units and source-scene identification.
- Walkable floor shapes or a derived navigation grid.
- Obstacle identities, geometry and relevant clearances.
- Entrances and traversability information.
- Validated pickup, delivery, spawn and waiting waypoints.
- Diagnostics for unknown objects, rejected waypoints and unreachable destinations.

Do not hard-code an environment snapshot as a second authority if the underlying approved `.blend` scene changes. Re-extract and validate the scene instead.

## 8. Validation checklist

Before relying on the exported map for autonomous delivery, verify:

- [ ] The approved scene remains unchanged by extraction.
- [ ] All **18 tables**, **72 chairs**, **6 pillars** and service points are accounted for, or discrepancies are explained.
- [ ] All **four entrances** and their actual openings are represented.
- [ ] Both connected floor arms and their shared boundary are handled correctly.
- [ ] Rotated and parented object transforms are exported in world coordinates.
- [ ] Unknown objects and invalid geometry produce explicit diagnostics.
- [ ] Pickup / delivery candidates have safe clearance and are reachable.
- [ ] Full-footprint collisions are rejected at different robot orientations.
- [ ] A Blender integration check confirms the Python map matches the scene.

The project plan tracks implementation of these checks in [phases.md](phases.md). Passing an isolated Python unit test is not the same as validating the **actual Blender scene**.
