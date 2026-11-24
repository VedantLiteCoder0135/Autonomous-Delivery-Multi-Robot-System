"""Deterministic battery accounting for the 20 Hz V14 simulation.

The coefficients are simulator assumptions, not measured hardware values.
Energy use and capacity fade are deliberately tracked independently.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class BatteryConfig:
    nominal_capacity_wh: float = 120.0
    initial_soc: float = 0.95
    movement_wh_per_m: float = 0.75
    rotation_wh_per_rad: float = 0.06
    idle_power_w: float = 12.0
    charging_power_w: float = 60.0
    charging_efficiency: float = 0.90
    cycle_fade_per_efc: float = 0.0005
    calendar_fade_per_day: float = 0.0
    minimum_soh: float = 0.60
    reserve_fraction: float = 0.15
    low_soc_fraction: float = 0.20
    return_soc_fraction: float = 0.15
    critical_soc_fraction: float = 0.10
    dispatch_soc_fraction: float = 0.80
    charge_target_fraction: float = 0.90
    protected_arrival_fraction: float = 0.05

    def __post_init__(self) -> None:
        positive = (self.nominal_capacity_wh, self.movement_wh_per_m,
                    self.charging_power_w)
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("capacity, movement coefficient, and charge power must be positive")
        nonnegative = (self.rotation_wh_per_rad, self.idle_power_w,
                       self.cycle_fade_per_efc, self.calendar_fade_per_day)
        if any(not math.isfinite(v) or v < 0 for v in nonnegative):
            raise ValueError("energy and degradation coefficients must be nonnegative")
        fractions = (self.initial_soc, self.charging_efficiency, self.minimum_soh,
                     self.reserve_fraction, self.low_soc_fraction,
                     self.return_soc_fraction, self.critical_soc_fraction,
                     self.dispatch_soc_fraction, self.charge_target_fraction,
                     self.protected_arrival_fraction)
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in fractions):
            raise ValueError("SOC, SOH, reserve, and efficiency values must be in [0, 1]")
        if self.minimum_soh <= 0:
            raise ValueError("minimum_soh must be positive")
        if not (self.critical_soc_fraction <= self.return_soc_fraction <= self.low_soc_fraction
                <= self.dispatch_soc_fraction <= self.charge_target_fraction):
            raise ValueError("battery SOC thresholds are inconsistent")
        if self.reserve_fraction + self.protected_arrival_fraction >= 1:
            raise ValueError("combined reserves must be less than usable capacity")


@dataclass(slots=True)
class BatteryState:
    robot_id: int
    remaining_wh: float
    soh: float = 1.0
    cumulative_discharged_wh: float = 0.0
    equivalent_full_cycles: float = 0.0
    last_sim_time_s: float = 0.0
    charged_wh: float = 0.0

    @property
    def usable_capacity_wh(self) -> float:
        return self._capacity_wh * self.soh

    @property
    def soc(self) -> float:
        capacity = self.usable_capacity_wh
        return min(1.0, self.remaining_wh / capacity) if capacity > 0 else 0.0

    @property
    def soc_percent(self) -> float:
        return 100.0 * self.soc

    @property
    def available_wh(self) -> float:
        return self.remaining_wh

    # Bound at creation to keep state records convenient and serializable.
    _capacity_wh: float = 120.0

    def to_dict(self) -> dict[str, float | int]:
        return {
            "robot_id": self.robot_id,
            "nominal_capacity_wh": self._capacity_wh,
            "remaining_wh": round(self.remaining_wh, 6),
            "soc_percent": round(self.soc_percent, 4),
            "soh": round(self.soh, 8),
            "usable_capacity_wh": round(self.usable_capacity_wh, 6),
            "cumulative_discharged_wh": round(self.cumulative_discharged_wh, 6),
            "equivalent_full_cycles": round(self.equivalent_full_cycles, 8),
            "charged_wh": round(self.charged_wh, 6),
        }


class BatteryManager:
    """Own battery state and exact motion/charging energy updates."""

    def __init__(self, robot_ids, config: BatteryConfig | None = None) -> None:
        self.config = config or BatteryConfig()
        self.states = {
            int(robot_id): BatteryState(
                int(robot_id), self.config.nominal_capacity_wh * self.config.initial_soc,
                _capacity_wh=self.config.nominal_capacity_wh,
            ) for robot_id in robot_ids
        }

    def get(self, robot_id: int) -> BatteryState:
        return self.states[int(robot_id)]

    def add_robot(self, robot_id: int) -> BatteryState:
        robot_id = int(robot_id)
        if robot_id in self.states:
            raise ValueError(f"battery state already exists for robot {robot_id}")
        state = BatteryState(robot_id, self.config.nominal_capacity_wh * self.config.initial_soc,
                             _capacity_wh=self.config.nominal_capacity_wh)
        self.states[robot_id] = state
        return state

    def _apply_discharge(self, state: BatteryState, energy_wh: float) -> float:
        if not math.isfinite(energy_wh) or energy_wh < 0:
            raise ValueError("discharged energy must be finite and nonnegative")
        used = min(state.remaining_wh, energy_wh)
        state.remaining_wh -= used
        state.cumulative_discharged_wh += used
        efc_delta = used / self.config.nominal_capacity_wh
        state.equivalent_full_cycles += efc_delta
        state.soh = max(self.config.minimum_soh,
                        state.soh - self.config.cycle_fade_per_efc * efc_delta)
        state.remaining_wh = min(state.remaining_wh, state.usable_capacity_wh)
        return used

    def update_motion(self, robot_id: int, distance_m: float, heading_change_rad: float,
                      dt_s: float, sim_time_s: float) -> float:
        """Deduct energy from observed pose deltas; return Wh actually discharged."""
        values = (distance_m, heading_change_rad, dt_s, sim_time_s)
        if any(not math.isfinite(v) for v in values) or min(values) < 0:
            raise ValueError("motion deltas and simulation time must be finite and nonnegative")
        state = self.get(robot_id)
        elapsed = max(0.0, sim_time_s - state.last_sim_time_s)
        state.last_sim_time_s = sim_time_s
        calendar_loss = self.config.calendar_fade_per_day * elapsed / 86400.0
        if calendar_loss:
            state.soh = max(self.config.minimum_soh, state.soh - calendar_loss)
            state.remaining_wh = min(state.remaining_wh, state.usable_capacity_wh)
        stationary = dt_s if distance_m <= 1e-9 and heading_change_rad <= 1e-9 else 0.0
        energy = (self.config.movement_wh_per_m * distance_m
                  + self.config.rotation_wh_per_rad * heading_change_rad
                  + self.config.idle_power_w * stationary / 3600.0)
        return self._apply_discharge(state, energy)

    def charge(self, robot_id: int, dt_s: float, *, docked: bool,
               stationary: bool, power_w: float | None = None) -> float:
        """Add energy only for an owned, docked, stationary robot; return Wh added."""
        if not math.isfinite(dt_s) or dt_s < 0:
            raise ValueError("charge timestep must be finite and nonnegative")
        if not docked or not stationary or dt_s == 0:
            return 0.0
        power = self.config.charging_power_w if power_w is None else power_w
        if not math.isfinite(power) or power < 0:
            raise ValueError("charge power must be finite and nonnegative")
        state = self.get(robot_id)
        capacity = state.usable_capacity_wh
        added = min(capacity - state.remaining_wh,
                    power * self.config.charging_efficiency * dt_s / 3600.0)
        state.remaining_wh += max(0.0, added)
        state.charged_wh += max(0.0, added)
        return max(0.0, added)

    def predicted_soh_loss(self, robot_id: int, discharged_wh: float) -> float:
        if discharged_wh < 0 or not math.isfinite(discharged_wh):
            raise ValueError("predicted discharge must be finite and nonnegative")
        return self.config.cycle_fade_per_efc * discharged_wh / self.config.nominal_capacity_wh

    def can_complete(self, robot_id: int, mission_wh: float) -> bool:
        if not math.isfinite(mission_wh) or mission_wh < 0:
            return False
        state = self.get(robot_id)
        reserve = self.config.reserve_fraction * state.usable_capacity_wh
        return state.soh > self.config.minimum_soh and mission_wh + reserve <= state.remaining_wh + 1e-9

    def return_soc_threshold(self, robot_id: int, energy_to_charger_wh: float) -> float:
        state = self.get(robot_id)
        required = energy_to_charger_wh + self.config.protected_arrival_fraction * state.usable_capacity_wh
        dynamic = 1.0 if state.usable_capacity_wh <= 0 else required / state.usable_capacity_wh
        return max(self.config.return_soc_fraction, dynamic)

    def telemetry(self) -> list[dict[str, float | int]]:
        return [self.states[robot_id].to_dict() for robot_id in sorted(self.states)]
