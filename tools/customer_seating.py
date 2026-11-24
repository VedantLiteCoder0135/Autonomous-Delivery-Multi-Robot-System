"""Member 3: chair-accurate, table-first random seating for the V14 cafeteria.

Uses exported scene geometry. Never changes the static occupancy grid or the
robot's delivery waypoint, and never invents additional chairs.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import defaultdict
import math
import random


FREE = "FREE"
RESERVED = "RESERVED"
OCCUPIED = "OCCUPIED"


@dataclass
class Seat:
    seat_id: str
    table_id: str
    x: float
    y: float
    heading: float
    approach_x: float
    approach_y: float
    delivery_x: float
    delivery_y: float
    status: str = FREE
    customer_id: int | None = None

    @property
    def position(self) -> tuple[float, float]:
        return self.x, self.y

    @property
    def approach(self) -> tuple[float, float]:
        return self.approach_x, self.approach_y


class SeatRegistry:
    """Atomic reservation and release in the single-threaded simulation tick.

    A table is sampled uniformly from tables with free chairs, then one of its
    free chairs uniformly. `rng` is injectable to make tests reproducible.
    """
    def __init__(self, env, grid, delivery_waypoints, reachable, *, rng=None,
                 expected_chairs=72):
        self.rng = rng if rng is not None else random.Random()
        self.seats: dict[str, Seat] = {}
        self.by_table: dict[str, list[str]] = defaultdict(list)
        self.customer_seats: dict[int, str] = {}
        self.unreachable_chairs: list[str] = []
        table_centers = {
            t.table_id: ((t.rect.min_x + t.rect.max_x)/2,
                         (t.rect.min_y + t.rect.max_y)/2)
            for t in env.tables
        }
        for chair in env.chairs:
            table = str(chair.parent_table)
            if table not in table_centers or table not in delivery_waypoints:
                self.unreachable_chairs.append(chair.name)
                continue
            x = (chair.rect.min_x + chair.rect.max_x)/2
            y = (chair.rect.min_y + chair.rect.max_y)/2
            tcx, tcy = table_centers[table]
            dx, dy = tcx-x, tcy-y
            heading = math.atan2(-dx, dy)  # Blender local +Y faces the table
            point = self._approach(grid, reachable, x, y, tcx, tcy)
            if point is None:
                self.unreachable_chairs.append(chair.name)
                continue
            # This is an existing validated robot waypoint, not the chair.
            waypoint = min(delivery_waypoints[table],
                           key=lambda p: math.hypot(p.x-x, p.y-y))
            seat = Seat(chair.name, table, x, y, heading, *point,
                        waypoint.x, waypoint.y)
            self.seats[seat.seat_id] = seat
            self.by_table[table].append(seat.seat_id)

        self.scene_chair_count = len(env.chairs)
        self.expected_chairs = expected_chairs
        self.chair_count_matches = self.scene_chair_count == expected_chairs
        self.unreachable_chairs.sort()
        for ids in self.by_table.values():
            ids.sort()

    @staticmethod
    def _approach(grid, reachable, x, y, tx, ty):
        # Face the chair from its outer side. Reject floor cells behind the
        # chair (towards the table); avoid selecting a point *inside* furniture.
        outward_x, outward_y = x-tx, y-ty
        norm = math.hypot(outward_x, outward_y)
        if norm < 1e-8:
            return None
        outward_x /= norm
        outward_y /= norm
        col, row = grid.world_to_grid(x, y)
        extent = int(math.ceil(1.1/grid.resolution)) + 2
        candidates = []
        for r in range(max(0,row-extent), min(grid.height,row+extent+1)):
            for c in range(max(0,col-extent), min(grid.width,col+extent+1)):
                if (c,r) not in reachable or not grid.is_free(c,r):
                    continue
                px,py = grid.grid_to_world(c,r)
                vx,vy = px-x,py-y
                d = math.hypot(vx,vy)
                outward = vx*outward_x+vy*outward_y
                if not (.25 <= d <= 1.05 and outward >= .10):
                    continue
                # Prefers an approach in front of the seat, not a diagonal
                # across furniture, with roughly 35 cm of space to stand.
                sideways = abs(-vx*outward_y+vy*outward_x)
                score = (abs(d-.42)+sideways*.70, d, px, py)
                candidates.append(score)
        if not candidates:
            return None
        _,_,px,py = min(candidates)
        return px,py

    def free_chairs(self, table_id: str) -> list[Seat]:
        return [self.seats[s] for s in self.by_table.get(table_id, ())
                if self.seats[s].status == FREE]

    def reserve_random_seat(self, customer_id: int, table_id: str | None = None) -> Seat | None:
        if customer_id in self.customer_seats:
            return self.seats[self.customer_seats[customer_id]]
        if table_id is not None and table_id != "AUTO":
            if table_id not in self.by_table:
                raise ValueError("Unknown or inaccessible table: " + table_id)
            tables = [table_id] if self.free_chairs(table_id) else []
        else:
            tables = sorted(t for t in self.by_table if self.free_chairs(t))
        if not tables:
            return None
        table = self.rng.choice(tables)
        seat = self.rng.choice(self.free_chairs(table))
        seat.status, seat.customer_id = RESERVED, customer_id
        self.customer_seats[customer_id] = seat.seat_id
        return seat

    def get(self, customer_id: int) -> Seat | None:
        seat_id = self.customer_seats.get(customer_id)
        return self.seats[seat_id] if seat_id is not None else None

    def occupy(self, customer_id: int) -> Seat:
        seat = self.get(customer_id)
        if seat is None:
            raise ValueError("Cannot occupy an unreserved chair")
        if seat.status not in (RESERVED, OCCUPIED):
            raise ValueError("Seat reservation has been lost")
        seat.status = OCCUPIED
        return seat

    def release(self, customer_id: int) -> Seat | None:
        seat_id = self.customer_seats.pop(customer_id, None)
        if seat_id is None:
            return None
        seat = self.seats[seat_id]
        if seat.customer_id != customer_id:
            raise AssertionError("Seat reservation ownership mismatch")
        seat.customer_id, seat.status = None, FREE
        return seat

    def counts(self) -> dict[str, int]:
        return {table: sum(self.seats[s].status != FREE for s in ids)
                for table, ids in sorted(self.by_table.items())}

    def audit(self) -> dict:
        return {
            "scene_chair_count": self.scene_chair_count,
            "expected_chair_count": self.expected_chairs,
            "expected_count_matches": self.chair_count_matches,
            "mapped_chairs": len(self.seats),
            "unreachable_or_unmapped_chairs": list(self.unreachable_chairs),
            "table_count": len(self.by_table),
            "seats_per_table": {t: len(ids) for t, ids in sorted(self.by_table.items())},
        }
