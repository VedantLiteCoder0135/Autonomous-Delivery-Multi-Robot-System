"""VS Code / Terminal validation entrypoint for integrated Stages 7–9."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from tools.stage79_engine import Stage79Engine, load_reference
from tools.stage79_scenarios import v14_pedestrian
import math


def main() -> int:
    parser = argparse.ArgumentParser(description='Run Stages 7–9 Python simulation on real cafeteria exports')
    parser.add_argument('--robots', type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument('--orders', nargs='+', default=['T11:SANDWICH', 'T12:JUICE', 'T13:TEA'],
                        metavar='TABLE:STATION')
    parser.add_argument('--max-ticks', type=int, default=6500)
    parser.add_argument('--project', type=Path, default=ROOT)
    parser.add_argument('--pedestrian', action='store_true',
                        help='Run a verified gentle moving-pedestrian T11 demo (1 robot)')
    args = parser.parse_args()
    if args.max_ticks < 1:
        parser.error('--max-ticks must be positive')
    if args.pedestrian and args.robots != 1:
        parser.error('--pedestrian validation currently supports --robots 1')
    env, grid, _ = load_reference(args.project)
    provider = v14_pedestrian(grid) if args.pedestrian else None
    engine = Stage79Engine(env, grid, robot_count=args.robots, pedestrians=provider)
    for value in args.orders:
        if ':' not in value:
            parser.error(f'Expected TABLE:STATION, got {value!r}')
        table, station = value.split(':', 1)
        engine.create_order(table, station)
    print(f'STAGES 7–9: {args.robots} robots, {len(args.orders)} orders, '
          f'{len(env.entrances)} wall-verified entrances', flush=True)
    report = engine.run(max_ticks=args.max_ticks)
    if args.pedestrian:
        baseline = Stage79Engine(env, grid, robot_count=1)
        for value in args.orders:
            table, station = value.split(':', 1)
            baseline.create_order(table, station)
        base_report = baseline.run(max_ticks=args.max_ticks)
        first = engine.robots[1].trace
        second = baseline.robots[1].trace
        max_deviation = max((math.dist((a['x'],a['y']),(b['x'],b['y']))
                             for a,b in zip(first,second)), default=0.)
        changed = sum(a['v'] != b['v'] or a['w'] != b['w']
                      for a,b in zip(first,second))
        report['pedestrian_comparison'] = {
            'baseline_success':base_report['success'],
            'max_position_deviation_m':round(max_deviation,6),
            'different_command_ticks':changed,
            'motion_affected':base_report['success'] and max_deviation > .02 and changed > 0,
        }
        report['success'] = bool(report['success'] and base_report['success']
                                 and report['pedestrian_comparison']['motion_affected']
                                 and report['min_pedestrian_clearance'] is not None
                                 and report['min_pedestrian_clearance'] > 0)
    outputs = args.project / 'outputs'
    outputs.mkdir(parents=True, exist_ok=True)
    (outputs / 'stage79_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    trace = {
        'dt': .05,
        'robots': {str(i): r.trace for i, r in engine.robots.items()},
        'routes': engine.routes,
        'history': engine.history,
    }
    (outputs / 'stage79_trace.json').write_text(json.dumps(trace), encoding='utf-8')
    print(json.dumps({
        'success': report['success'],
        'simulation_ticks': report['ticks'],
        'simulated_seconds': report['time_s'],
        'entrances_classified': len(report['entrances']),
        'delivered_orders': sum(o['status'] == 'DELIVERED' for o in report['orders']),
        'robots_at_base_and_available': sum(r['phase'] == 'IDLE' and r['available'] for r in report['robots']),
        'return_before_reassignment': report['return_before_reassignment'],
        'alns_initial_cost': report['alns_total_initial_cost'],
        'alns_optimized_cost': report['alns_total_optimized_cost'],
        'minimum_robot_clearance_m': report['min_robot_clearance'],
        'minimum_pedestrian_clearance_m': report['min_pedestrian_clearance'],
        'pedestrian_comparison': report.get('pedestrian_comparison'),
        'failures': report['failures'],
    }, indent=2))
    print('Wrote outputs/stage79_report.json and outputs/stage79_trace.json')
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
