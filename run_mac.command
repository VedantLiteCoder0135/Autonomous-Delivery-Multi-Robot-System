#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
BLENDER="/Applications/Blender.app/Contents/MacOS/Blender"
if [[ ! -x "$BLENDER" ]]; then
  echo "Blender not found at $BLENDER. Install Blender or update BLENDER in this script."; exit 1
fi
SCENE="$PWD/AIFR_Cafeteria_V14_Recovered.blend"
[[ -f "$SCENE" ]] || { echo "Missing recovered cafeteria scene"; exit 1; }
"$BLENDER" "$SCENE" --background --python-expr "import bpy; assert len(bpy.data.objects)>650, 'Cafeteria scene is empty or incomplete'; print('SCENE_OBJECTS:',len(bpy.data.objects))"
"$BLENDER" "$SCENE" --background --python "$PWD/tools/blender_stage16_bridge.py"
python3 - <<'PY2'
import json
from pathlib import Path
r=json.loads(Path('outputs/stage36_report.json').read_text())
assert r['success'] and r['mission']['completed'] and r['mission']['trace_footprint_clear'] and r['stage2']['final_order_state']=='DELIVERED',r
assert r['counts']['tables']==18 and r['counts']['chairs']==72 and r['counts']['pillars']==6,r['counts']
print('PASS: V14 cafeteria and Stages 1–6 test mission')
PY2
