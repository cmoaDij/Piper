"""Local browser dashboard for frames and non-blocking policy diagnostics."""
import json
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
from urllib.parse import parse_qs, urlsplit

import cv2


PAGE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>UMI Piper dashboard</title><style>
body{font-family:sans-serif;margin:18px;background:#16191d;color:#eee}
.views{display:flex;flex-wrap:wrap;gap:14px}.card{background:#222831;padding:12px;border-radius:8px;flex:1;min-width:280px}
img{width:100%;height:auto;background:#333}#model,#reference,#overlay{max-width:448px;width:100%;image-rendering:pixelated}
label{display:inline-block;margin:8px 18px 8px 0}select,input{vertical-align:middle}small{display:block;overflow-wrap:anywhere;color:#bbb}
pre{white-space:pre-wrap;font-size:12px;line-height:1.35}.plot{width:100%;height:170px;background:#111}
.ok{color:#7ee787}.warn{color:#f2cc60}.bad{color:#ff7b72}
</style><h2>UMI Piper dashboard</h2>
<div class="views"><section class="card"><h3>Camera frame</h3><img id="raw"></section>
<section class="card"><h3>Model input (224 × 224)</h3><img id="model"></section></div>
<section id="alignment" hidden><h3>训练首帧与实时画面对齐</h3>
<label>示教片段 <select id="episode"></select></label>
<label>训练图像透明度 <input id="alpha" type="range" min="0" max="1" step="0.05"><output id="alpha-value"></output></label>
<small id="reference-source"></small><p>0% 为实时画面，100% 为训练首帧。叠加仅用于显示，模型输入和动作保持原样。</p>
<div class="views"><section class="card"><h3>Training episode first frame</h3><img id="reference"></section>
<section class="card"><h3>Live + training overlay</h3><canvas id="overlay" width="448" height="448"></canvas></section></div></section>
<div class="views"><section class="card"><h3>Runtime status</h3><pre id="status">Waiting for events...</pre></section>
<section class="card"><h3>TCP X: predicted / actual</h3><canvas id="plot" class="plot"></canvas></section></div>
<p>关闭页面不会停止策略；请在终端使用 Ctrl+C 停止运行。</p>
<script>
const live=document.getElementById('model'),reference=document.getElementById('reference'),alpha=document.getElementById('alpha'),episode=document.getElementById('episode');
function overlay(){if(!live.complete||!live.naturalWidth||!reference.complete||!reference.naturalWidth)return;const c=document.getElementById('overlay'),x=c.getContext('2d');x.globalAlpha=1;x.clearRect(0,0,c.width,c.height);x.drawImage(live,0,0,c.width,c.height);x.globalAlpha=Number(alpha.value);x.drawImage(reference,0,0,c.width,c.height);x.globalAlpha=1}
function opacity(){document.getElementById('alpha-value').value=Math.round(Number(alpha.value)*100)+'%';overlay()}
live.addEventListener('load',overlay);reference.addEventListener('load',overlay);alpha.addEventListener('input',opacity);
episode.addEventListener('change',()=>{reference.src='/reference.jpg?episode='+episode.value});
fetch('/reference.json').then(r=>r.json()).then(s=>{if(!s.enabled)return;document.getElementById('alignment').hidden=false;for(let i=0;i<s.episode_count;i++){let option=document.createElement('option');option.value=i;option.textContent='第 '+(i+1)+' 段（episode '+i+'）';episode.appendChild(option)}episode.value=s.initial_episode;alpha.value=s.initial_alpha;document.getElementById('reference-source').textContent=s.dataset;opacity();reference.src='/reference.jpg?episode='+s.initial_episode}).catch(()=>{});
function draw(s){const c=document.getElementById('plot'),x=c.getContext('2d'),w=c.clientWidth*devicePixelRatio,h=c.clientHeight*devicePixelRatio;c.width=w;c.height=h;x.clearRect(0,0,w,h);x.strokeStyle='#555';x.strokeRect(0,0,w,h);let a=(s.tcp_history||[]);if(!a.length)return;let vals=a.flatMap(v=>[v.predicted_x,v.actual_x]).filter(Number.isFinite),lo=Math.min(...vals),hi=Math.max(...vals);if(hi-lo<1e-6){hi=lo+1e-6}for(const [key,color] of [['predicted_x','#58a6ff'],['actual_x','#7ee787']]){x.strokeStyle=color;x.beginPath();a.forEach((v,i)=>{let xx=i/(a.length-1||1)*w,yy=h-(v[key]-lo)/(hi-lo)*h;i?x.lineTo(xx,yy):x.moveTo(xx,yy)});x.stroke()}}
function refresh(){let t=Date.now();for(const id of ['raw','model'])document.getElementById(id).src='/'+id+'.jpg?t='+t;fetch('/status.json?t='+t).then(r=>r.json()).then(s=>{let cls=s.level||'ok';let p=document.getElementById('status');p.className=cls;p.textContent=JSON.stringify(s,null,2);draw(s)}).catch(()=>{})}refresh();setInterval(refresh,500)
</script></html>'''
PAGE = PAGE.encode('utf-8')


class CameraPreview:
    def __init__(self, port, *, training_reference=None, match_episode=0, match_alpha=.5):
        self.lock = threading.Lock()
        self.frames = None
        self.status = {'level': 'warn', 'message': 'Waiting for policy events'}
        self.tcp_history = []
        self.reference_metadata = {'enabled': False}
        self.reference_jpegs = []
        if training_reference is not None:
            self.reference_metadata = training_reference.metadata(match_episode, match_alpha)
            for rgb in training_reference.images_rgb:
                ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                              [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise ValueError('Could not encode training reference')
                self.reference_jpegs.append(encoded.tobytes())
        preview = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                request = urlsplit(self.path)
                path = request.path
                if path == '/':
                    self.reply(200, 'text/html; charset=utf-8', PAGE)
                    return
                if path == '/status.json':
                    with preview.lock:
                        status = dict(preview.status)
                        status['tcp_history'] = list(preview.tcp_history)
                    self.reply(200, 'application/json', json.dumps(status).encode('utf-8'))
                    return
                if path == '/reference.json':
                    self.reply(200, 'application/json', json.dumps(preview.reference_metadata).encode('utf-8'))
                    return
                if path == '/reference.jpg' and preview.reference_jpegs:
                    try:
                        episode = int(parse_qs(request.query).get('episode', ['0'])[0])
                        if not 0 <= episode < len(preview.reference_jpegs):
                            raise ValueError('episode out of range')
                    except ValueError:
                        self.send_error(400, 'Invalid episode index')
                        return
                    self.reply(200, 'image/jpeg', preview.reference_jpegs[episode])
                    return
                if path not in ('/raw.jpg', '/model.jpg'):
                    self.send_error(404)
                    return
                with preview.lock:
                    frames = preview.frames
                if frames is None:
                    self.send_error(503, 'Waiting for camera frames')
                    return
                image = frames[0] if path == '/raw.jpg' else cv2.cvtColor(frames[1], cv2.COLOR_RGB2BGR)
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    self.send_error(500, 'Could not encode frame')
                    return
                self.reply(200, 'image/jpeg', jpeg.tobytes())

            def reply(self, status, content_type, content):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(content)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, format, *args):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    @property
    def port(self):
        return self.server.server_address[1]

    def start(self):
        self.thread.start()

    def update(self, raw_bgr, model_rgb):
        with self.lock:
            self.frames = (raw_bgr, model_rgb)

    def update_status(self, status):
        """Update dashboard state; this only replaces a small in-memory object."""
        if not isinstance(status, dict):
            return
        with self.lock:
            current = dict(self.status)
            current.update(status)
            self.status = current
            predicted = current.get('predicted_flange_xyz_rpy')
            actual = current.get('actual_flange_xyz_rpy')
            if isinstance(predicted, (list, tuple)) and len(predicted) >= 1 \
                    and isinstance(actual, (list, tuple)) and len(actual) >= 1:
                self.tcp_history.append({'predicted_x': float(predicted[0]),
                                         'actual_x': float(actual[0])})
                del self.tcp_history[:-120]

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


class CameraReader:
    """Keep draining capture independently of browser requests and inference."""
    def __init__(self, device, recorder=None):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open camera; close other capture applications')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frames = deque(maxlen=120)
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.error = None
        self.recorder = recorder
        self.thread = threading.Thread(target=self._read, daemon=True)

    def start(self):
        self.thread.start()

    def _read(self):
        try:
            while not self.stopped.is_set():
                ok, frame = self.cap.read()
                if not ok:
                    raise RuntimeError('Camera failed to deliver a frame')
                timestamp = time.monotonic()
                if self.recorder is not None:
                    self.recorder.write(timestamp, frame)
                with self.lock:
                    self.frames.append((timestamp, frame))
        except Exception as exc:
            self.error = exc

    def recent(self):
        with self.lock:
            return list(self.frames)

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=2)
        self.cap.release()
