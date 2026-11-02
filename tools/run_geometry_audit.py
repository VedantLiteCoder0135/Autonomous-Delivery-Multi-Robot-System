"""Generate a Phase 1 navigation-geometry audit from committed V14 exports."""
from pathlib import Path
import json

from tools.geometry_audit import audit_environment
from tools.stage79_engine import load_reference


def main():
    project=Path(__file__).resolve().parents[1]
    env,grid,_=load_reference(project)
    report=audit_environment(env,grid)
    dest=project/'outputs'/'phase1_geometry_audit.json'
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(report,indent=2)+'\n')
    print(f"PHASE 1 GEOMETRY AUDIT: {len(report['pickup_stations'])} stations, "
          f"{len(report['waiting_candidates'])} waiting candidates")
    for s in report['pickup_stations']:
        print(f"  {s['station']}: {s['candidate_feasible']}/{len(s['pickup_poses'])} "
              "pickup approach candidates passed conservative checks")
        for p in s['pickup_poses']:
            if p['status']!='CANDIDATE_FEASIBLE':
                print(f"    REVIEW {p['label']}: {p['status']}")
    print(f'Saved {dest}')
    print('NOTE: steering/turning and real corridor non-blocking behavior remain unverified.')


if __name__=='__main__':
    main()
