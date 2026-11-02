"""Blender-independent verification and sampling of a Stage 7–9 trace."""
from __future__ import annotations

import json
import math
from pathlib import Path

SOURCE_SCENE = 'AIFR_Cafeteria_V14_Recovered.blend'
ALLOWED_PHASES = {'IDLE', 'WAITING_FOR_AISLE', 'TO_PICKUP', 'LOADING',
                  'TO_CUSTOMER', 'UNLOADING', 'RETURNING', 'FAULT'}


def load_and_validate(project: Path) -> tuple[dict, dict]:
    project = Path(project)
    scene = project / SOURCE_SCENE
    if not scene.is_file():
        raise FileNotFoundError(f'Missing recovered V14 scene: {scene}')
    out = project / 'outputs'
    report = json.loads((out / 'stage79_report.json').read_text(encoding='utf8'))
    trace = json.loads((out / 'stage79_trace.json').read_text(encoding='utf8'))
    if report.get('success') is not True or report.get('failures'):
        raise ValueError('Python simulation failed; refusing Blender visualization')
    if report.get('return_before_reassignment') is not True:
        raise ValueError('Return-before-reassignment verification missing')
    entrances = report.get('entrances', [])
    if len(entrances) != 4 or len({e['name'] for e in entrances}) != 4:
        raise ValueError('Expected four uniquely classified entrances')
    if not isinstance(trace.get('dt'), (float, int)) or trace['dt'] <= 0:
        raise ValueError('Missing positive trace time step')
    traces = trace.get('robots')
    if not isinstance(traces, dict) or not traces:
        raise ValueError('Trace contains no robot trajectories')
    robot_ids = {int(r['id']) for r in report.get('robots', [])}
    if robot_ids != {int(i) for i in traces}:
        raise ValueError('Report and trace robot IDs differ')
    if not all(r.get('phase') == 'IDLE' and r.get('available') is True
               for r in report['robots']):
        raise ValueError('Not all robots returned to base')
    if not report.get('orders') or any(o.get('status') != 'DELIVERED' for o in report['orders']):
        raise ValueError('Not all orders were delivered')
    max_tick = int(report.get('ticks', -1))
    if max_tick <= 0:
        raise ValueError('Invalid simulation length')
    for rid, samples in traces.items():
        if not isinstance(samples, list) or len(samples) < 2:
            raise ValueError(f'Robot {rid}: missing samples')
        last_tick = -1
        for s in samples:
            tick = s.get('tick')
            if not isinstance(tick, int) or tick < last_tick or tick > max_tick:
                raise ValueError(f'Robot {rid}: invalid or unordered tick')
            last_tick = tick
            if s.get('phase') not in ALLOWED_PHASES:
                raise ValueError(f'Robot {rid}: unknown phase')
            if not all(isinstance(s.get(k), (int, float)) and math.isfinite(s[k])
                       for k in ('x', 'y', 'theta', 'v', 'w')):
                raise ValueError(f'Robot {rid}: non-finite pose/velocity')
        if samples[-1]['phase'] != 'IDLE':
            raise ValueError(f'Robot {rid}: final state is not IDLE')
    for route in trace.get('routes', []):
        if int(route.get('robot_id', -1)) not in robot_ids:
            raise ValueError('Route refers to unknown robot')
        if not isinstance(route.get('path'), list):
            raise ValueError('Route has no path')
    return report, trace


def sample_robot(samples: list[dict], stride: int = 5) -> list[dict]:
    """Keep phase/order boundaries and regularly spaced ticks for keyframing."""
    if stride < 1:
        raise ValueError('stride must be positive')
    selected = []
    for i, item in enumerate(samples):
        prior = samples[i - 1] if i else item
        nxt = samples[i + 1] if i + 1 < len(samples) else item
        transition = (item['phase'], item.get('order_id')) != (prior['phase'], prior.get('order_id'))
        last_of_phase = (item['phase'], item.get('order_id')) != (nxt['phase'], nxt.get('order_id'))
        if i == 0 or i == len(samples) - 1 or i % stride == 0 or transition or last_of_phase:
            selected.append(item)
    return selected


def animation_frame(sample: dict, dt: float, fps: int = 30) -> int:
    # Tick zero is first frame, avoid assuming every recorded robot has tick zero.
    return 1 + round(sample['tick'] * dt * fps)
