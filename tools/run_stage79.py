"""VS Code / Terminal validation entrypoint for integrated Stages 7–9."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
for folder in (ROOT, ROOT / 'src'):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from tools.stage79_engine import Stage79Engine, load_reference
from tools.stage79_scenarios import v14_pedestrian
from aifr.battery import BatteryConfig
import math


def main() -> int:
    parser = argparse.ArgumentParser(description='Run Stages 7–9 Python simulation on real cafeteria exports')
    parser.add_argument('--robots', type=int, default=2, choices=(1, 2, 3, 4))
    parser.add_argument('--orders', nargs='+', default=['T11:SANDWICH', 'T12:JUICE', 'T13:TEA'],
                        metavar='TABLE:STATION')
    parser.add_argument('--max-ticks', type=int, default=6500)
    parser.add_argument('--project', type=Path, default=ROOT)
    parser.add_argument('--battery-enabled', action='store_true',
                        help='Enable simulated battery state and mission feasibility gating')
    parser.add_argument('--intelligent-allocation', action='store_true',
                        help='Use Hungarian matching (requires --battery-enabled)')
    parser.add_argument('--charging-enabled', action='store_true',
                        help='Use statically audited existing base slots as charger pads')
    parser.add_argument('--capacity-wh', type=float, default=120.0)
    parser.add_argument('--movement-wh-per-m', type=float, default=0.75)
    parser.add_argument('--rotation-wh-per-rad', type=float, default=0.06)
    parser.add_argument('--idle-power-w', type=float, default=12.0)
    parser.add_argument('--charging-power-w', type=float, default=60.0)
    parser.add_argument('--cycle-fade-per-efc', type=float, default=0.0005)
    parser.add_argument('--calendar-fade-per-day', type=float, default=0.0)
    parser.add_argument('--reserve-fraction', type=float, default=0.15)
    parser.add_argument('--pedestrian', action='store_true',
                        help='Run a verified gentle moving-pedestrian T11 demo (1 robot)')
    args = parser.parse_args()
    if args.max_ticks < 1:
        parser.error('--max-ticks must be positive')
    if args.pedestrian and args.robots != 1:
        parser.error('--pedestrian validation currently supports --robots 1')
    if args.intelligent_allocation and not args.battery_enabled:
        parser.error('--intelligent-allocation requires --battery-enabled')
    if args.charging_enabled and not args.battery_enabled:
        parser.error('--charging-enabled requires --battery-enabled')
    try:
        battery_config = BatteryConfig(
            nominal_capacity_wh=args.capacity_wh,
            movement_wh_per_m=args.movement_wh_per_m,
            rotation_wh_per_rad=args.rotation_wh_per_rad,
            idle_power_w=args.idle_power_w,
            charging_power_w=args.charging_power_w,
            cycle_fade_per_efc=args.cycle_fade_per_efc,
            calendar_fade_per_day=args.calendar_fade_per_day,
            reserve_fraction=args.reserve_fraction)
    except ValueError as exc:
        parser.error(str(exc))
    env, grid, _ = load_reference(args.project)
    provider = v14_pedestrian(grid) if args.pedestrian else None
    engine = Stage79Engine(
        env, grid, robot_count=args.robots, pedestrians=provider,
        battery_enabled=args.battery_enabled,
        intelligent_allocation_enabled=args.intelligent_allocation,
        charging_enabled=args.charging_enabled,
        battery_config=battery_config)
    for value in args.orders:
        if ':' not in value:
            parser.error(f'Expected TABLE:STATION, got {value!r}')
        table, station = value.split(':', 1)
        engine.create_order(table, station)
    print(f'STAGES 7–9: {args.robots} robots, {len(args.orders)} orders, '
          f'{len(env.entrances)} wall-verified entrances', flush=True)
    report = engine.run(max_ticks=args.max_ticks)
    if args.pedestrian:
        baseline = Stage79Engine(env, grid, robot_count=1,
                                 battery_enabled=args.battery_enabled,
                                 intelligent_allocation_enabled=args.intelligent_allocation,
                                 charging_enabled=args.charging_enabled,
                                 battery_config=battery_config)
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
        'battery_management_enabled': report['battery_management_enabled'],
        'intelligent_allocation_enabled': report['intelligent_allocation_enabled'],
        'charging_enabled': report['charging_enabled'],
        'energy_discharged_wh': report['total_energy_discharged_wh'],
        'energy_charged_wh': report['total_energy_charged_wh'],
        'unfinished_orders': report['unfinished_orders'],
        'pedestrian_comparison': report.get('pedestrian_comparison'),
        'failures': report['failures'],
    }, indent=2))
    print('Wrote outputs/stage79_report.json and outputs/stage79_trace.json')
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
