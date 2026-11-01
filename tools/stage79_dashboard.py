"""Stage 9 localhost dashboard for Python-only simulation verification.

No Blender dependency, no external Python packages, no network binding beyond
127.0.0.1. The engine remains the authority for scheduling and motion.
"""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from tools.stage79_engine import Stage79Engine, load_reference


HTML = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>AIFR | Python Control Center</title>
<style>
:root{font-family:Inter,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color:#17212e;background:#f3f5f7}
*{box-sizing:border-box}body{margin:0}.top{background:#142337;color:#fff;padding:18px 28px;display:flex;align-items:center;justify-content:space-between;gap:14px;flex-wrap:wrap}
.brand{font-weight:800;font-size:19px;letter-spacing:.03em}.sub{font-size:12px;color:#b7c5d6;margin-top:5px}.pill{background:#233c54;border-radius:20px;padding:7px 12px;font-size:12px}
.layout{display:grid;grid-template-columns:minmax(420px,1.65fr) minmax(330px,1fr);gap:18px;padding:18px;max-width:1500px;margin:auto}
.panel{background:#fff;border:1px solid #e1e6ec;border-radius:13px;box-shadow:0 2px 8px #152e4710;padding:18px;margin-bottom:18px}
h2{font-size:16px;margin:0 0 14px}h3{font-size:13px;margin:15px 0 8px}.small{font-size:12px;color:#617080}
canvas{width:100%;height:auto;background:#f7f8fa;border:1px solid #e3e9ef;border-radius:8px;display:block}
.controls{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px}button,select,input{font:inherit;border-radius:7px;border:1px solid #cbd5df;padding:9px 11px;background:white;color:#17212e}
button{cursor:pointer;font-size:12px;font-weight:700}button:hover{background:#edf2f7}.primary{background:#1664ae;color:#fff;border-color:#1664ae}.primary:hover{background:#104f89}.danger{background:#9d2930;color:white;border-color:#9d2930}.danger:hover{background:#842028}
form{display:flex;flex-wrap:wrap;gap:8px}select{min-width:95px;flex:1}form button{flex:1}
.statrow{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.stat{background:#f3f6f9;border-radius:8px;padding:12px}.stat strong{display:block;font-size:19px;margin-top:4px}
table{border-collapse:collapse;width:100%;font-size:12px}th,td{text-align:left;border-bottom:1px solid #edf0f3;padding:8px 4px}th{color:#6a7787;font-weight:600}tr:last-child td{border:0}
.events{max-height:160px;overflow:auto;font-size:11px;color:#526272;font-family:ui-monospace,monospace;line-height:1.8}
#message{font-size:12px;min-height:17px;margin-top:10px} .good{color:#17734d}.bad{color:#ae3030}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:11px;color:#617080;margin-top:10px}.sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px}
@media(max-width:900px){.layout{grid-template-columns:1fr;padding:10px}.top{padding:16px}.panel{padding:13px}}
</style></head><body>
<header class="top"><div><div class="brand">AIFR / ROBOT OPERATIONS</div><div class="sub">Stages 7–9 · Local Python simulation · Cafeteria V14</div></div><span class="pill" id="live">Connecting…</span></header>
<div class="layout"><section><div class="panel"><h2>Cafeteria navigation map</h2><canvas id="map" width="900" height="710"></canvas>
<div class="legend"><span><i class="sw" style="background:#cbd5de"></i>Tables / chairs</span><span><i class="sw" style="background:#33465a"></i>Walls</span><span><i class="sw" style="background:#2974af"></i>Food counters</span><span><i class="sw" style="background:#e59a27"></i>Entrances</span><span><i class="sw" style="background:#168c71"></i>Robot traces</span></div>
<p class="small">Uses the extracted Stage 1–6 grid and geometry. Movement is calculated by Python; this is not Blender physics.</p></div>
<div class="panel"><h2>Order queue</h2><table><thead><tr><th>Order</th><th>Table</th><th>Station</th><th>Status</th><th>Robot</th></tr></thead><tbody id="orders"></tbody></table></div></section>
<aside><div class="panel"><h2>Simulation controls</h2><div class="controls"><button class="primary" onclick="control('start')">Start / Resume</button><button onclick="control('pause')">Pause</button><button onclick="control('step')">Step</button><button class="danger" onclick="control('estop')">Emergency stop</button><button onclick="control('reset')">Reset</button><button onclick="control('save')">Save report</button></div>
<label class="small" for="speed">Playback speed</label> <select id="speed" onchange="control('speed',+this.value)"><option value="1">1×</option><option value="2">2×</option><option value="4">4×</option><option value="8">8×</option><option value="16">16×</option></select>
<div id="message"></div></div>
<div class="panel"><h2>New food order</h2><form id="create"><select id="table" aria-label="Table"></select><select id="station" aria-label="Food station"></select><button class="primary" type="submit">Add READY order</button></form><p class="small">Dispatch follows READY-time FCFS. Robots cannot be reassigned until they return to base.</p></div>
<div class="panel"><h2>Live metrics</h2><div class="statrow"><div class="stat"><span class="small">Simulation time</span><strong id="clock">0s</strong></div><div class="stat"><span class="small">Ready queue</span><strong id="queued">0</strong></div><div class="stat"><span class="small">Entrances</span><strong id="entrances">0/4</strong></div></div>
<h3>Robot fleet</h3><table><thead><tr><th>Robot</th><th>Phase</th><th>Order</th><th>Available</th></tr></thead><tbody id="robots"></tbody></table>
<h3>Recent events</h3><div class="events" id="events"></div></div>
<div class="panel"><h2>Verification</h2><div class="small" id="verification">Waiting for simulation.</div></div></aside></div>
<script>
let scene=null, state=null; const canvas=document.getElementById('map'), ctx=canvas.getContext('2d');
const C=['#168c71','#ce7341','#7958ba','#2f6fb1'];
function el(id){return document.getElementById(id)}
async function request(url,body){const o=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};const r=await fetch(url,o);const data=await r.json();if(!r.ok)throw Error(data.error||'Request failed');return data}
function message(t,bad=false){el('message').textContent=t;el('message').className=bad?'bad':'good'}
async function control(action,value){try{const x=await request('/api/control',{action,value});message(x.message||action);refresh()}catch(e){message(e.message,true)}}
el('create').addEventListener('submit',async e=>{e.preventDefault();try{const x=await request('/api/order',{table:el('table').value,station:el('station').value,ready:true});message('Created order #'+x.order_id);refresh()}catch(e){message(e.message,true)}});
function rect(r,fill){ctx.fillStyle=fill;const x=X(r.min_x),y=Y(r.max_y),w=(r.max_x-r.min_x)*scale,h=(r.max_y-r.min_y)*scale;ctx.fillRect(x,y,w,h)}
let scale=1;function X(x){return 22+(x-scene.origin[0])*scale}function Y(y){return canvas.height-22-(y-scene.origin[1])*scale}
function dot(x,y,r,color){ctx.fillStyle=color;ctx.beginPath();ctx.arc(X(x),Y(y),r,0,2*Math.PI);ctx.fill()}
function draw(){if(!scene||!state)return;ctx.clearRect(0,0,canvas.width,canvas.height);scale=Math.min((canvas.width-44)/scene.width,(canvas.height-44)/scene.height);
for(const f of scene.floors)rect(f.rect,'#edf2ee');for(const a of scene.chairs)rect(a.rect,'#d4dde5');for(const a of scene.tables)rect(a.rect,'#b9c6d1');for(const a of scene.counters)rect(a.rect,a.is_food_station?'#2974af':'#8a9bab');for(const a of scene.pillars)rect(a.rect,'#5d6974');for(const a of scene.walls)rect(a.rect,'#33465a');
ctx.font='bold 11px sans-serif';for(const e of scene.entrances){dot(e.center_x,e.center_y,5,'#e59a27');ctx.fillStyle='#9c6516';ctx.fillText(e.name.replaceAll('_',' '),X(e.center_x)+7,Y(e.center_y)-5)}
for(const [i,r] of state.robots.entries()){const col=C[i%C.length],trace=state.traces[String(r.id)]||[];if(trace.length){ctx.strokeStyle=col+'a0';ctx.lineWidth=2;ctx.beginPath();trace.forEach((p,j)=>j?ctx.lineTo(X(p[0]),Y(p[1])):ctx.moveTo(X(p[0]),Y(p[1])));ctx.stroke()}
ctx.strokeStyle=col;ctx.lineWidth=2;ctx.beginPath();ctx.arc(X(r.x),Y(r.y),9,0,Math.PI*2);ctx.stroke();dot(r.x,r.y,5,col);ctx.beginPath();ctx.moveTo(X(r.x),Y(r.y));ctx.lineTo(X(r.x)+14*Math.cos(r.theta),Y(r.y)-14*Math.sin(r.theta));ctx.stroke();ctx.fillStyle='#16283b';ctx.fillText('R'+r.id,X(r.x)+12,Y(r.y)-10)} }
function rows(id,items){el(id).innerHTML=items.join('')}
function esc(s){return String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
async function refresh(){try{state=await request('/api/state');el('live').textContent=state.estopped?'EMERGENCY STOP':state.paused?'PAUSED':'RUNNING';el('clock').textContent=state.time_s.toFixed(1)+'s';el('queued').textContent=state.queue.length;el('entrances').textContent=(scene?scene.entrances.length:0)+'/4';el('speed').value=String(state.speed);
rows('orders',state.orders.map(o=>`<tr><td>#${o.id}</td><td>${esc(o.table)}</td><td>${esc(o.station)}</td><td>${esc(o.status)}</td><td>${o.robot_id??'—'}</td></tr>`));
rows('robots',state.robots.map(r=>`<tr><td>R${r.id}</td><td>${esc(r.phase)}</td><td>${r.order_id??'—'}</td><td>${r.fleet_available?'Yes':'No'}</td></tr>`));
el('events').innerHTML=state.recent_events.slice(-12).reverse().map(e=>`<div>[${e.tick}] ${esc(e.event)} ${e.robot_id?'R'+e.robot_id:''} ${e.order_id?'#'+e.order_id:''}</div>`).join('');
let status=state.failures.length?'FAULT: '+state.failures.join('; '):'No simulation faults';el('verification').textContent=status+' · '+state.routes_count+' ALNS routes · '+state.yield_events+' traffic yield events';draw()}catch(e){el('live').textContent='DISCONNECTED';message(e.message,true)}}
async function boot(){try{scene=await request('/api/map');for(const id of ['table','station']){const options=id==='table'?scene.tables_list:scene.stations_list;el(id).innerHTML=options.map(v=>`<option value="${esc(v)}">${esc(v)}</option>`).join('')}el('table').value='T11';el('station').value='SANDWICH';refresh();setInterval(refresh,500)}catch(e){message(e.message,true)}}boot();
</script></body></html>'''


class Stage79App:
    def __init__(self, project: Path, robot_count: int = 2):
        self.project = project
        self.env, self.grid, _ = load_reference(project)
        self.robot_count = robot_count
        self.lock = threading.RLock()
        self.engine = Stage79Engine(self.env, self.grid, robot_count)
        self.closed = threading.Event()

    def map_data(self):
        return {
            'origin': [self.grid.origin_x, self.grid.origin_y],
            'width': self.grid.width * self.grid.resolution,
            'height': self.grid.height * self.grid.resolution,
            'floors': [x.to_dict() for x in self.env.floor_regions],
            'walls': [x.to_dict() for x in self.env.walls],
            'pillars': [x.to_dict() for x in self.env.pillars],
            'tables': [x.to_dict() for x in self.env.tables],
            'chairs': [x.to_dict() for x in self.env.chairs],
            'counters': [x.to_dict() for x in self.env.counters],
            'entrances': [x.to_dict() for x in self.env.entrances],
            'tables_list': sorted(self.engine.drops),
            'stations_list': sorted(self.engine.pickups),
        }

    def state(self):
        with self.lock:
            e = self.engine
            result = e.snapshot()
            result['traces'] = {str(i): [[p['x'], p['y']] for p in r.trace[::8]]
                                for i, r in e.robots.items()}
            result['routes_count'] = len(e.routes)
            return result

    def order(self, data):
        with self.lock:
            if self.engine.estopped:
                raise ValueError('Reset after emergency stop before adding orders')
            return {'order_id': self.engine.create_order(str(data['table']),
                                                         str(data['station']),
                                                         bool(data.get('ready', True)))}

    def control(self, data):
        action = data.get('action')
        with self.lock:
            e = self.engine
            if action == 'reset':
                self.engine = Stage79Engine(self.env, self.grid, self.robot_count)
                return {'message': 'Simulation reset. Existing orders cleared.'}
            if action == 'estop':
                e.estopped = True
                e.paused = True
                return {'message': 'Emergency stop latched. Reset required.'}
            if e.estopped:
                raise ValueError('Emergency stop latched. Reset to continue.')
            if action in ('start', 'resume'):
                e.paused = False
                return {'message': 'Simulation running'}
            if action == 'pause':
                e.paused = True
                return {'message': 'Simulation paused'}
            if action == 'step':
                e.paused = False
                try:
                    e.step()
                finally:
                    e.paused = True
                return {'message': 'Advanced one 50 ms simulation tick'}
            if action == 'speed':
                value = float(data['value'])
                if value not in (1., 2., 4., 8., 16.):
                    raise ValueError('Speed must be 1, 2, 4, 8, or 16')
                e.speed = value
                return {'message': f'Speed set to {value:g}×'}
            if action == 'save':
                output = self.project / 'outputs'
                output.mkdir(exist_ok=True, parents=True)
                (output / 'stage79_dashboard_report.json').write_text(
                    json.dumps(e.report(), indent=2), encoding='utf-8')
                return {'message': 'Saved outputs/stage79_dashboard_report.json'}
            raise ValueError(f'Unknown action: {action}')

    def tick_loop(self):
        while not self.closed.is_set():
            with self.lock:
                e = self.engine
                if not e.paused and not e.estopped and not e.failures:
                    if e.orders.all_orders() and all(o.status.value == 'DELIVERED'
                                                      for o in e.orders.all_orders()) and all(
                            r.phase == 'IDLE' for r in e.robots.values()):
                        e.paused = True
                    else:
                        e.step()
                delay = .05 / e.speed if not e.paused else .05
            self.closed.wait(delay)


def make_handler(app: Stage79App):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def send(self, code, obj, content_type='application/json'):
            raw = (obj.encode('utf-8') if isinstance(obj, str)
                   else json.dumps(obj, allow_nan=False).encode('utf-8'))
            self.send_response(code)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == '/':
                self.send(200, HTML, 'text/html')
            elif self.path == '/api/state':
                self.send(200, app.state())
            elif self.path == '/api/map':
                self.send(200, app.map_data())
            elif self.path == '/api/report':
                with app.lock:
                    self.send(200, app.engine.report())
            else:
                self.send(404, {'error': 'Not found'})

        def do_POST(self):
            if self.path not in ('/api/order', '/api/control'):
                self.send(404, {'error': 'Not found'})
                return
            try:
                n = int(self.headers.get('Content-Length', '0'))
                if n < 1 or n > 8192:
                    raise ValueError('Request body must be 1–8192 bytes')
                data = json.loads(self.rfile.read(n))
                if not isinstance(data, dict):
                    raise ValueError('Expected JSON object')
                result = app.order(data) if self.path == '/api/order' else app.control(data)
                self.send(200, result)
            except (ValueError, KeyError, TypeError) as exc:
                self.send(400, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description='Stage 9 localhost Python dashboard')
    parser.add_argument('--robots', type=int, choices=(1, 2, 3, 4), default=2)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    app = Stage79App(ROOT, args.robots)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(app))
    worker = threading.Thread(target=app.tick_loop, daemon=True)
    worker.start()
    print(f'AIFR Python dashboard: http://127.0.0.1:{server.server_port}', flush=True)
    print('Ctrl+C to exit. Localhost only; no Blender needed.', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        app.closed.set()
        server.server_close()


if __name__ == '__main__':
    main()
