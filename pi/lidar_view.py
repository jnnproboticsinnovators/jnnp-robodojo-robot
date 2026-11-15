#!/usr/bin/env python3
"""
JNNP RoboDojo - see what the LiDAR sees, live in the laptop browser.

On the Pi:   python3 ~/lidar_view.py
On laptop:   open  http://<PI-IP>:8001   in Chrome
Ctrl+C on the Pi to stop.

Needs lidar_a1.py in the same folder. Do not run it at the same time as the teleop
(only one program can use the LiDAR at a time).
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from lidar_a1 import A1Lidar

PORT = 8001
latest = {'points': [], 'turn': 0, 'status': 'starting (3 s)...'}
lock = threading.Lock()

PAGE = b"""<!doctype html><html><head><meta charset='utf-8'><title>LiDAR view</title>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<style>body{margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center}
canvas{background:#000;max-width:96vw;max-height:80vh}#info{margin:8px;font-size:15px}
button{font-size:15px;margin:2px 4px;padding:4px 10px}</style></head><body>
<h3 style='margin:8px'>JNNP RoboDojo - what the LiDAR sees (top view)</h3>
<div>Range: <button onclick='R=1'>1 m</button><button onclick='R=2'>2 m</button>
<button onclick='R=4'>4 m</button><button onclick='R=8'>8 m</button></div>
<canvas id='c' width='800' height='800'></canvas><div id='info'>connecting...</div>
<script>
let R=2;const c=document.getElementById('c'),g=c.getContext('2d'),C=400,S=380;
function draw(d){g.clearRect(0,0,800,800);
 g.strokeStyle='#333';g.fillStyle='#777';g.font='13px sans-serif';
 const step=R<=1?0.25:R<=2?0.5:R<=4?1:2;
 for(let r=step;r<=R+1e-6;r+=step){g.beginPath();g.arc(C,C,r/R*S,0,6.3);g.stroke();g.fillText(r.toFixed(2)+' m',C+4,C-r/R*S+14);}
 for(let a=0;a<360;a+=30){const t=(a-90)*Math.PI/180;g.beginPath();g.moveTo(C,C);g.lineTo(C+S*Math.cos(t),C+S*Math.sin(t));g.stroke();
  g.fillText(a+'\\u00b0',C+(S+10)*Math.cos(t)-10,C+(S+10)*Math.sin(t)+4);}
 g.strokeStyle='#0c0';g.lineWidth=2;g.beginPath();g.moveTo(C,C);g.lineTo(C,C-60);g.stroke();g.lineWidth=1;
 g.fillStyle='#0c0';g.fillText('0\\u00b0 (LiDAR front)',C+6,C-50);
 g.fillStyle='#09f';g.fillRect(C-6,C-6,12,12);
 let near=null;
 for(const [a,dist] of d.points){if(dist>R)continue;const t=(a-90)*Math.PI/180;
  const x=C+dist/R*S*Math.cos(t),y=C+dist/R*S*Math.sin(t);
  g.fillStyle=dist<0.2?'#f33':dist<0.5?'#fa0':'#fff';g.fillRect(x-2,y-2,4,4);
  if(!near||dist<near[1])near=[a,dist];}
 document.getElementById('info').textContent=
  `LiDAR: ${d.status} | turn ${d.turn} | ${d.points.length} points`+
  (near?` | nearest ${near[1].toFixed(2)} m at ${near[0].toFixed(0)}\\u00b0`:'')+
  ' | red < 0.2 m, orange < 0.5 m';}
async function loop(){try{const r=await fetch('/scan.json');draw(await r.json());}
 catch(e){document.getElementById('info').textContent='no connection to the Pi';}setTimeout(loop,200);}
loop();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/scan.json':
            with lock:
                body = json.dumps(latest).encode()
            ctype = 'application/json'
        elif self.path == '/':
            body, ctype = PAGE, 'text/html'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


def lidar_thread():
    import time
    while True:
        lidar = A1Lidar()
        try:
            with lock:
                latest['status'] = 'starting (3 s)...'
            if not lidar.start():
                raise IOError('no reply')
            for n, turn in enumerate(lidar.scans(), 1):
                with lock:
                    latest['points'] = [[round(a, 1), round(d, 3)] for a, d in turn]
                    latest['turn'] = n
                    latest['status'] = f'OK (auto-restarts {lidar.restarts})'
        except Exception as e:
            with lock:
                latest['status'] = f'error ({e}) - retrying'
            time.sleep(1)
        finally:
            lidar.close()


if __name__ == '__main__':
    threading.Thread(target=lidar_thread, daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f'LiDAR view: open  http://<PI-IP>:{PORT}  in the laptop browser.  Ctrl+C to stop.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    print('Stopped.')
