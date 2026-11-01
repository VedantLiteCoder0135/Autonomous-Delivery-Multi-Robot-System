"""Randomized scheduling tests against an independent, deliberately naive reference model.

The reference re-derives every expected assignment from scratch (sort the ready list, sort the
free robots, zip). It shares no code with the real queue, ledger, registry or scheduler.
Random traffic includes ties, batches, cancellations, failures, robot faults and recoveries.
"""

from __future__ import annotations

import random

import pytest

from aifr.fleet import RobotStatus
from aifr.orders import FLOW, OrderStatus as S

from helpers import Core, build_core

IN_FLIGHT = (S.ASSIGNED, S.TO_PICKUP, S.LOADING, S.TO_CUSTOMER, S.UNLOADING)
CANCELLABLE = (S.CREATED, S.PREPARING, S.READY, S.ASSIGNED, S.TO_PICKUP)
NON_TERMINAL = (S.CREATED, S.PREPARING, S.READY) + IN_FLIGHT


class Reference:
    def __init__(self, robots: int) -> None:
        self.free = set(range(1, robots + 1))
        self.faulted: set[int] = set()
        self.busy: dict[int, int] = {}                # robot -> order
        self.ready: list[tuple[int, int, int]] = []   # (ready_at, seq, order)
        self.seq = 0

    def mark_ready(self, order: int, at: int) -> None:
        self.seq += 1
        self.ready.append((at, self.seq, order))

    def drop(self, order: int) -> None:
        self.ready = [e for e in self.ready if e[2] != order]
        for robot, o in list(self.busy.items()):
            if o == order:
                del self.busy[robot]
                if robot not in self.faulted:
                    self.free.add(robot)

    def dispatch(self) -> list[tuple[int, int]]:
        self.ready.sort()
        out = []
        for robot in sorted(self.free):
            if not self.ready:
                break
            _, _, order = self.ready.pop(0)
            self.free.discard(robot)
            self.busy[robot] = order
            out.append((order, robot))
        return out

    def fault(self, robot: int) -> None:
        if robot in self.faulted:
            return
        self.faulted.add(robot)
        self.free.discard(robot)
        self.busy.pop(robot, None)                    # its order fails

    def recover(self, robot: int) -> None:
        self.faulted.discard(robot)
        self.free.add(robot)


def check_invariants(core: Core, ref: Reference) -> None:
    core.scheduler.ledger.check_consistency()
    ledger = core.scheduler.ledger
    busy = {int(r.robot_id) for r in core.fleet.all() if r.status is RobotStatus.BUSY}
    assert busy == set(ref.busy) == {int(a.robot_id) for a in ledger.active()}
    assert [int(r) for r in core.fleet.available_ids()] == sorted(ref.free)
    expected_queue = [o for _, _, o in sorted(ref.ready)]
    assert [int(o) for o in core.scheduler.queue.order_ids()] == expected_queue
    in_flight = {int(o.order_id) for o in core.orders.by_status(*IN_FLIGHT)}
    assert in_flight == {int(a.order_id) for a in ledger.active()} == set(ref.busy.values())
    assert not (set(expected_queue) & in_flight)      # never queued and assigned at once
    history = [int(a.order_id) for a in ledger.history]
    assert len(history) == len(set(history))          # no order ever assigned twice


def run_scenario(seed: int, robots: int, steps: int = 220) -> int:
    rng = random.Random(seed)
    core, ref = build_core(robots), Reference(robots)
    total_assigned = 0

    for _ in range(steps):
        if rng.random() < 0.5:
            core.clock.advance(rng.randint(1, 3))     # often no advance -> READY timestamp ties
        for _ in range(rng.randint(0, 2)):
            core.orders.create_order(rng.choice(["T03", "T11", "T22"]), rng.choice(["TEA", "JUICE"]))
        for o in core.orders.by_status(S.CREATED):
            if rng.random() < 0.6:
                core.orders.start_preparing(o.order_id)
        preparing = core.orders.by_status(S.PREPARING)
        rng.shuffle(preparing)
        for o in preparing[: rng.randint(0, 4)]:
            core.orders.mark_ready(o.order_id)
            ref.mark_ready(int(o.order_id), core.clock.now())

        for o in core.orders.by_status(*IN_FLIGHT):
            if rng.random() < 0.35:
                nxt = FLOW[FLOW.index(o.status) + 1]
                core.orders.advance(o.order_id, nxt)
                if nxt is S.DELIVERED:
                    ref.drop(int(o.order_id))

        if rng.random() < 0.10:
            pool = core.orders.by_status(*CANCELLABLE)
            if pool:
                victim = rng.choice(pool)
                core.orders.cancel(victim.order_id)
                ref.drop(int(victim.order_id))
        if rng.random() < 0.06:
            pool = core.orders.by_status(*NON_TERMINAL)
            if pool:
                victim = rng.choice(pool)
                core.orders.fail(victim.order_id)
                ref.drop(int(victim.order_id))
        if rng.random() < 0.06:
            robot = rng.randint(1, robots)
            core.fleet.mark_faulted(robot)  # type: ignore[arg-type]
            ref.fault(robot)
        if ref.faulted and rng.random() < 0.20:
            robot = rng.choice(sorted(ref.faulted))
            core.fleet.recover(robot)  # type: ignore[arg-type]
            ref.recover(robot)

        if rng.random() < 0.7:                        # sometimes skip -> bigger batches
            actual = [(int(a.order_id), int(a.robot_id)) for a in core.scheduler.dispatch()]
            expected = ref.dispatch()
            assert actual == expected, f"seed={seed} robots={robots} tick={core.clock.tick}"
            total_assigned += len(actual)
        check_invariants(core, ref)

    return total_assigned


@pytest.mark.parametrize("robots", range(1, 9))
@pytest.mark.parametrize("seed", range(25))
def test_scheduler_matches_reference_model(seed, robots):
    run_scenario(seed, robots)


def test_random_scenarios_actually_exercise_the_scheduler():
    """Guard against a vacuous pass: scenarios must assign a meaningful number of orders."""
    assigned = [run_scenario(seed, robots) for seed in range(5) for robots in (1, 3, 8)]
    assert min(assigned) > 5 and sum(assigned) > 300
