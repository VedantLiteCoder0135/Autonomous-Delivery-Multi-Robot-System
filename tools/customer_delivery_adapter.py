"""Connect authoritative Stage 7 delivery events to Member 3 meal timers.

The live integration owner calls this once after each engine step. This bridge
never mutates orders or robot scheduler state; delivery remains authoritative.
"""
from __future__ import annotations

import time


class CustomerDeliveryBridge:
    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.cursor = 0

    def consume(self, engine, flow, *, now_real_s=None) -> int:
        events = engine.events[self.cursor:]
        self.cursor = len(engine.events)
        accepted = 0
        if not flow.customer_seating_enabled:
            return accepted
        for event in events:
            if event.get('event') != 'DELIVERED_RETURN_PENDING':
                continue
            order_id = int(event['order_id'])
            customer_id = engine.order_customers.get(order_id)
            if customer_id is None:
                continue
            timestamp = float(self.clock() if now_real_s is None else now_real_s)
            if flow.on_food_delivered(order_id, customer_id, now_real_s=timestamp):
                if flow.on_food_served(order_id, customer_id, now_real_s=timestamp):
                    accepted += 1
        return accepted
