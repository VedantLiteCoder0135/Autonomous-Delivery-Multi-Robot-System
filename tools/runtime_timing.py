"""Pure helpers for Blender-native real-time stepping (no Blender import).

Simulation time always advances in fixed 0.05 s ticks. Rendering and input
polling remain responsive even when requested simulation speed exceeds CPU.
"""
from __future__ import annotations

def requested_steps(multiplier: int | str, refresh_interval: float = 0.1,
                    physics_dt: float = 0.05) -> int:
    speed = int(multiplier)
    if speed not in (1, 2, 4, 8, 16):
        raise ValueError('speed must be 1, 2, 4, 8, or 16')
    if refresh_interval <= 0 or physics_dt <= 0:
        raise ValueError('time increments must be positive')
    return max(1,round(speed*refresh_interval/physics_dt))
