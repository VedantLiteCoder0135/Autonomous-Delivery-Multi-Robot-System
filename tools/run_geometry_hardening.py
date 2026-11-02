"""Run Phase 1 egress and waiting accessibility analysis."""
from pathlib import Path
import json
from tools.stage79_engine import load_reference
from tools.geometry_hardening import audit_egress_and_waiting

def main():
    root=Path(__file__).resolve().parents[1]
    env,grid,_=load_reference(root)
    result=audit_egress_and_waiting(env,grid)
    out=root/'outputs'/'geometry_egress_waiting.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(result,indent=2))
    print(f"EGRESS: {sum(x['egress_reachable'] for x in result['pickups'])}/{len(result['pickups'])} pickup routes; "
          f"WAITING: {sum(x['nonblocking_for_pickup_access'] for x in result['waiting'])}/{len(result['waiting'])} candidates preserve pickup access")
    print('Report:',out)
if __name__=='__main__':main()
