"""Member 3: idempotent delivery/serving clock for a single customer order.

Only a verified served event starts an eating deadline. Wall seconds are
injected for tests, rather than the accelerated Stage 7 simulation clock.
"""
from __future__ import annotations

from dataclasses import dataclass
import random
import time


@dataclass
class Meal:
    customer_id: int
    order_id: int
    delivered_at_real_s: float | None = None
    served_at_real_s: float | None = None
    eating_duration_s: float | None = None
    deadline_real_s: float | None = None

    @property
    def delivered(self) -> bool:
        return self.delivered_at_real_s is not None

    @property
    def served(self) -> bool:
        return self.served_at_real_s is not None

    def expired(self, now_real_s: float) -> bool:
        return self.deadline_real_s is not None and now_real_s >= self.deadline_real_s


class MealClock:
    def __init__(self, *, rng=None, clock=None):
        self.rng = rng if rng is not None else random.Random()
        self.clock = clock if clock is not None else time.monotonic
        self.meals: dict[int, Meal] = {}

    def track(self, customer_id: int, order_id: int) -> Meal:
        current = self.meals.get(customer_id)
        if current is not None:
            if current.order_id != order_id:
                raise ValueError("Customer/order meal mismatch")
            return current
        meal = Meal(customer_id, order_id)
        self.meals[customer_id] = meal
        return meal

    def delivered(self, order_id: int, customer_id: int, *, now_real_s=None) -> bool:
        meal = self.meals.get(customer_id)
        if meal is None or meal.order_id != order_id:
            return False
        if meal.delivered:
            return False
        meal.delivered_at_real_s = float(self.clock() if now_real_s is None else now_real_s)
        return True

    def served(self, order_id: int, customer_id: int, *, now_real_s=None) -> bool:
        meal = self.meals.get(customer_id)
        if meal is None or meal.order_id != order_id or not meal.delivered:
            return False
        if meal.served:
            return False
        now = float(self.clock() if now_real_s is None else now_real_s)
        if now < meal.delivered_at_real_s:
            raise ValueError("Serving cannot precede delivery")
        meal.served_at_real_s = now
        meal.eating_duration_s = float(self.rng.uniform(20.0, 40.0))
        meal.deadline_real_s = now + meal.eating_duration_s
        return True

    def remove(self, customer_id: int) -> None:
        self.meals.pop(customer_id, None)
