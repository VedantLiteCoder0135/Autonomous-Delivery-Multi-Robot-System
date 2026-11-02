"""Reproducible integrated live-input acceptance scenarios for the V14 cafeteria.

Run: python -m tools.verify_hardening --scenario identical
     python -m tools.verify_hardening --scenario left_side
     python -m tools.verify_hardening --scenario four_customers

This uses the same step-by-step engine and cashier flow as Blender. CPU runtime
can be much longer than simulated time, especially with several robots.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

from tools.stage79_engine import Stage79Engine,load_reference
from tools.customer_flow import CustomerFlow

SCENARIOS={
  'identical': [('main_entry','T11','SANDWICH'),
                ('main_entry','T11','SANDWICH'),
                ('main_entry','T11','TEA')],
  'left_side': [('main_entry','T11','SANDWICH'),
                ('main_entry','T11','MAIN_COURSE')],
  'four_customers': [('main_entry','T11','SANDWICH')]*4,
}

def simulate(scenario='identical',max_ticks=10000,progress=False):
    project=Path(__file__).resolve().parents[1]
    env,grid,_=load_reference(project)
    ref={}
    engine=Stage79Engine(env,grid,robot_count=2,
                         pedestrians=lambda t:ref['flow'](t),traffic_mode='exclusive')
    flow=CustomerFlow(engine);ref['flow']=flow
    ids=[]
    for entrance,table,station in SCENARIOS[scenario]:
        ids.append(flow.add(entrance=entrance,table=table,station=station))
    engine.paused=False
    done=False
    for tick in range(max_ticks):
        flow.step();engine.step()
        if progress and tick%1000==0:
            print(f'[{scenario}] tick {tick}, orders {len(engine.orders.all_orders())}, '
                  f'robot states {[r.phase for r in engine.robots.values()]}',flush=True)
        if engine.failures:break
        orders=engine.orders.all_orders()
        if len(orders)==len(ids) and all(o.status.value=='DELIVERED' for o in orders) and all(
                r.phase=='IDLE' for r in engine.robots.values()):
            done=True;break
    customers=[flow.customers[i] for i in ids]
    order_ids=[c.order_id for c in customers]
    distinct=len(set(order_ids))==len(ids) and None not in order_ids
    linked=all(engine.order_customers.get(c.order_id)==c.customer_id for c in customers if c.order_id)
    events=engine.events
    pending=set(); lifecycle_valid=True
    for event in events:
        rid=event.get('robot_id')
        if event['event']=='DELIVERED_RETURN_PENDING':
            if rid in pending:lifecycle_valid=False
            pending.add(rid)
        elif event['event']=='RETURNED_TO_BASE':
            if rid not in pending:lifecycle_valid=False
            pending.discard(rid)
        elif event['event']=='ASSIGNED' and rid in pending:
            lifecycle_valid=False
    result={
        'scenario':scenario,'success':done and distinct and linked and lifecycle_valid
                   and not engine.failures and all(c.phase=='SEATED' for c in customers),
        'ticks':engine.clock.tick,'customers':len(ids),'created_orders':len(engine.orders.all_orders()),
        'distinct_order_ids':distinct,'customer_linkage_valid':linked,
        'return_before_reassignment':lifecycle_valid and not pending,
        'order_statuses':engine.snapshot(compact=True)['orders'],
        'robots':engine.snapshot(compact=True)['robots'],
        'failures':engine.failures,
        'limits':'Exclusive aisle policy, simple kinematic pedestrians; no general deadlock-freedom proof.'
    }
    out=project/'outputs'/f'validation_{scenario}.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario',choices=list(SCENARIOS),default='identical')
    parser.add_argument('--max-ticks',type=int,default=10000)
    args=parser.parse_args()
    report=simulate(args.scenario,args.max_ticks,progress=True)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['success'] else 1)
