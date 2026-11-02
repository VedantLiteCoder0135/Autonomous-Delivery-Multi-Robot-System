"""Robot registry: owns robot state and availability. Robot ids are 1..N."""

from __future__ import annotations

from aifr.common.clock import Clock
from aifr.common.errors import ConfigError, InvalidRobotStateError, UnknownRobotError
from aifr.common.events import EventBus
from aifr.common.types import OrderId, RobotId
from aifr.config.models import MAX_ROBOTS
from aifr.fleet.events import RobotAvailable, RobotFaulted, RobotStatusChanged
from aifr.fleet.robot import Robot, RobotStatus


class RobotRegistry:
    def __init__(self, bus: EventBus, clock: Clock, robot_count: int) -> None:
        if isinstance(robot_count, bool) or not isinstance(robot_count, int):
            raise ConfigError(f"robot_count must be an integer, got {robot_count!r}")
        if not 1 <= robot_count <= MAX_ROBOTS:
            raise ConfigError(f"robot_count must be in 1..{MAX_ROBOTS}, got {robot_count}")
        self._bus = bus
        self._clock = clock
        self._robots: dict[RobotId, Robot] = {
            RobotId(i): Robot(RobotId(i)) for i in range(1, robot_count + 1)
        }

    # ------------------------------------------------------------------ queries
    def get(self, robot_id: RobotId) -> Robot:
        try:
            return self._robots[robot_id]
        except KeyError:
            raise UnknownRobotError(f"unknown robot {robot_id}") from None

    def all(self) -> list[Robot]:
        """All robots sorted by id."""
        return [self._robots[i] for i in sorted(self._robots)]

    def available_ids(self) -> list[RobotId]:
        """Ids of currently available robots, ascending."""
        return [r.robot_id for r in self.all() if r.is_available]

    def is_available(self, robot_id: RobotId) -> bool:
        return self.get(robot_id).is_available

    def __len__(self) -> int:
        return len(self._robots)

    def add_robot(self) -> RobotId:
        """Register one new IDLE robot without disturbing existing assignments.

        The navigation layer must validate a physical base slot before calling.
        """
        new_id = RobotId(len(self._robots) + 1)
        if len(self._robots) >= MAX_ROBOTS:
            raise ConfigError(f"robot limit {MAX_ROBOTS} reached")
        self._robots[new_id] = Robot(new_id)
        self._bus.publish(RobotAvailable(self._clock.now(), new_id))
        return new_id

    # ------------------------------------------------------------- state changes
    def assign(self, robot_id: RobotId, order_id: OrderId) -> None:
        """IDLE -> BUSY with ``order_id``. Intended to be called by the scheduler only."""
        robot = self.get(robot_id)
        if not robot.is_available:
            raise InvalidRobotStateError(
                f"robot {robot_id} is not available (status={robot.status.value}, "
                f"active_order={robot.active_order_id})"
            )
        robot.status = RobotStatus.BUSY
        robot.active_order_id = order_id
        self._bus.publish(
            RobotStatusChanged(self._clock.now(), robot_id, RobotStatus.IDLE, RobotStatus.BUSY)
        )

    def release(self, robot_id: RobotId) -> None:
        """Clear the active order. A healthy robot becomes available immediately.

        No return-to-base step exists: availability depends on state, not location.
        A FAULT robot keeps its FAULT status and does not become available.
        """
        robot = self.get(robot_id)
        if robot.active_order_id is None:
            raise InvalidRobotStateError(f"robot {robot_id} has no active order to release")
        robot.active_order_id = None
        if robot.status is RobotStatus.BUSY:
            now = self._clock.now()
            robot.status = RobotStatus.IDLE
            with self._bus.defer():
                self._bus.publish(
                    RobotStatusChanged(now, robot_id, RobotStatus.BUSY, RobotStatus.IDLE)
                )
                self._bus.publish(RobotAvailable(now, robot_id))

    def mark_faulted(self, robot_id: RobotId) -> None:
        """Take a robot out of service. Idempotent."""
        robot = self.get(robot_id)
        if robot.status is RobotStatus.FAULT:
            return
        old = robot.status
        robot.status = RobotStatus.FAULT
        now = self._clock.now()
        with self._bus.defer():
            self._bus.publish(RobotStatusChanged(now, robot_id, old, RobotStatus.FAULT))
            self._bus.publish(RobotFaulted(now, robot_id))

    def recover(self, robot_id: RobotId) -> None:
        """FAULT -> IDLE, available again. Requires that its order was already released."""
        robot = self.get(robot_id)
        if robot.status is not RobotStatus.FAULT:
            raise InvalidRobotStateError(f"robot {robot_id} is not faulted")
        if robot.active_order_id is not None:
            raise InvalidRobotStateError(
                f"robot {robot_id} still holds order {robot.active_order_id}"
            )
        robot.status = RobotStatus.IDLE
        now = self._clock.now()
        with self._bus.defer():
            self._bus.publish(
                RobotStatusChanged(now, robot_id, RobotStatus.FAULT, RobotStatus.IDLE)
            )
            self._bus.publish(RobotAvailable(now, robot_id))
