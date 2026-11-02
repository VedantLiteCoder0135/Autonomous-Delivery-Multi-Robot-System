"""Pure-Python tests for the Stage 7–9 Blender trace validation layer."""
from __future__ import annotations
import json
from pathlib import Path
import pytest
from tools.blender_data_export import (SOURCE_SCENE, animation_frame,
                                       load_and_validate, sample_robot)


def fixtures(tmp_path: Path):
    (tmp_path / SOURCE_SCENE).write_bytes(b'blend')
    out = tmp_path / 'outputs'
    out.mkdir()
    report = {'success': True, 'return_before_reassignment': True,
              'failures': [], 'ticks': 5,
              'entrances': [{'name': n, 'center_x': 1, 'center_y': 2}
                            for n in ('a', 'b', 'c', 'd')],
              'orders': [{'status': 'DELIVERED'}],
              'robots': [{'id': 1, 'phase': 'IDLE', 'available': True}]}
    def sample(t, phase):
        return {'tick': t, 'x': 2., 'y': 3., 'theta': 0.,
                'v': 0., 'w': 0., 'phase': phase, 'order_id': 1}
    trace = {'dt': .05, 'robots': {'1': [sample(0, 'TO_PICKUP'),
                                          sample(1, 'TO_PICKUP'),
                                          sample(2, 'RETURNING'),
                                          sample(5, 'IDLE')]},
             'routes': [{'robot_id': 1, 'path': [[0, 0], [1, 1]]}]}
    return out, report, trace


def save(out, r, t):
    (out / 'stage79_report.json').write_text(json.dumps(r))
    (out / 'stage79_trace.json').write_text(json.dumps(t))


def test_valid_report_and_trace(tmp_path):
    out, r, t = fixtures(tmp_path)
    save(out, r, t)
    report, trace = load_and_validate(tmp_path)
    assert len(report['entrances']) == 4
    assert len(trace['robots']['1']) == 4


def test_failed_simulation_rejected(tmp_path):
    out, r, t = fixtures(tmp_path)
    r['success'] = False
    save(out, r, t)
    with pytest.raises(ValueError, match='simulation failed'):
        load_and_validate(tmp_path)


def test_missing_return_rejected(tmp_path):
    out, r, t = fixtures(tmp_path)
    r['return_before_reassignment'] = False
    save(out, r, t)
    with pytest.raises(ValueError, match='Return-before'):
        load_and_validate(tmp_path)


def test_invalid_entrance_count_rejected(tmp_path):
    out, r, t = fixtures(tmp_path)
    r['entrances'].pop()
    save(out, r, t)
    with pytest.raises(ValueError, match='four'):
        load_and_validate(tmp_path)


def test_nonfinite_pose_rejected(tmp_path):
    out, r, t = fixtures(tmp_path)
    t['robots']['1'][1]['x'] = float('nan')
    save(out, r, t)
    with pytest.raises(ValueError, match='non-finite'):
        load_and_validate(tmp_path)


def test_robot_mismatch_rejected(tmp_path):
    out, r, t = fixtures(tmp_path)
    r['robots'][0]['id'] = 2
    save(out, r, t)
    with pytest.raises(ValueError, match='IDs differ'):
        load_and_validate(tmp_path)


def test_sample_preserves_phase_changes():
    samples = [dict(tick=i, phase=('TO_PICKUP' if i < 3 else 'RETURNING'), order_id=1)
               for i in range(8)]
    result = sample_robot(samples, stride=5)
    assert [s['tick'] for s in result] == [0, 2, 3, 5, 7]
    assert animation_frame({'tick': 20}, .05, 30) == 31


def test_stride_must_be_positive():
    with pytest.raises(ValueError):
        sample_robot([], stride=0)
