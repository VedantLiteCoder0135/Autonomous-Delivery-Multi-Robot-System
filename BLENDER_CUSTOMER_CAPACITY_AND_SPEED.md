# Blender-native customer capacity and simulation speed patch

This is an **incremental update** to the existing Git checkout with the Blender-native customer-flow patch installed. There is no HTTP server and no separate repository.

## Updates
- Up to **4 customers per table**. Each customer reserves a distinct walkable seat anchor at least 0.40 m apart from other anchors at the same table. These are avatar positions near tables, not chair meshes.
- Up to **72 registered customers** across the 18 mapped tables (including people queued at entrances). The actual number that can walk comfortably at once is lower and depends on routing and computational load.
- **All four entrances support a FIFO queue**. If an entrance is occupied by a customer or robot, adding a customer queues them rather than rejecting the request. Queued customers are not displayed as overlapping avatars, and are admitted once the entrance is clear. Auto entrance selection favors fewer queued customers.
- Blender AIFR sidebar offers **1x, 2x, 4x, 8x, 16x** speed buttons. Speed multiplies the number of fixed 0.05 s simulation steps per timer callback. It does **not** change robot motion parameters, bypass collision guards, or guarantee wall-clock speed at high CPU load.
- Every customer's payment still creates a distinct Stage 2 order and marks it READY upon seating. Return-before-reassignment remains in place.

## Install in your existing Git repository

```bash
cd ~/Desktop/Projects/Autonomous-Delivery-Multi-Robot-System
git status --short
# Back up or commit edits to the three files this patch replaces first.
unzip -o ~/Downloads/AIFR_Blender_Customer_Capacity_Speed.zip -d .
source .venv/bin/activate
python -m pytest -q tests/test_customer_flow.py tests/test_live_runtime.py
```

Launch **Blender directly**, no localhost:

```bash
/Applications/Blender.app/Contents/MacOS/Blender \
  "$(pwd)/AIFR_Cafeteria_V14_Recovered.blend" \
  --python "$(pwd)/tools/blender_integration_live.py"
```

In the Blender viewport press `N`, then choose the **AIFR** tab. Click Add Customer four times for the same table and entrance. Customers should queue and enter in order. Try adding a fifth at that table; it should report the 4-person limit. Click 2x, 4x, 8x or 16x while running to increase simulated time per render update.

## Important limits
- Customer paths are currently grid-based and **not a full pedestrian-to-pedestrian collision-avoidance system**. Queuing is only enforced at entrances, not along aisles or the CASH counter.
- The scene shows customer avatars near tables, not models physically sitting in chairs.
- Very dense crowds can block robots or trigger navigation stalls. A 16x button requests accelerated simulation; Blender may run slower if computations take longer than the timer interval.
- The Blender GUI has **not been run in this development environment**. Test it on your Mac before committing or deleting the previous working copy.
