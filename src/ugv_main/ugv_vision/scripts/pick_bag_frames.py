#!/usr/bin/env python3
"""Hand-pick frames from camera bags in the browser (companion of extract_bag_frames.py).

Usage (source ROS 2 Humble first):
  pick_bag_frames.py --out dataset/frames Videos/*/
then open http://localhost:8000 (VS Code forwards the port over Remote-SSH).

Keys: Left / Right 1 frame, Up / Down 10, PageUp / PageDown 100, Space play / pause,
      S save, U unsave, N / P next / previous saved frame. Click the strip to jump.
Strip: green = saved, orange = turning, grey = blurred (same rules as extract_bag_frames.py).
Saved frames are the original JPEGs in <out>/<bag_name>/, named like extract_bag_frames.py
does, so its picks show up as saved and can be removed here.
"""
import argparse
import glob
import json
import os
import sqlite3
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import extract_bag_frames as ebf  # noqa: E402

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Frame picker</title>
<style>
body{margin:0;background:#111;color:#ddd;font:14px sans-serif}
#bar{padding:6px 10px;display:flex;gap:12px;align-items:center}
#strip{width:100%;height:24px;display:block;cursor:pointer}
#img{display:block;max-width:100vw;max-height:calc(100vh - 70px);margin:0 auto}
.tag{padding:2px 6px;border-radius:3px;color:#fff}
</style></head><body>
<div id="bar"><select id="bag"></select><span id="info"></span><span id="tags"></span>
<b id="count"></b><span style="margin-left:auto;color:#888">&larr;/&rarr; 1 &middot; &uarr;/&darr; 10 &middot;
PgUp/PgDn 100 &middot; Space play &middot; S save &middot; U unsave &middot; N/P next/prev saved</span></div>
<canvas id="strip" height="24"></canvas><img id="img">
<script>
let bag = 0, frames = [], saved = new Set(), k = 0, timer = null;
const $ = id => document.getElementById(id);
const url = j => `/img?bag=${bag}&k=${j}`;
const tag = (text, color) => `<span class="tag" style="background:${color}">${text}</span> `;

async function loadBags() {
  const names = await (await fetch('/api/bags')).json();
  names.forEach((n, i) => $('bag').add(new Option(n, i)));
  $('bag').onchange = () => { $('bag').blur(); openBag(+$('bag').value); };
  openBag(0);
}
async function openBag(i) {
  stop(); bag = i; frames = []; $('info').textContent = 'scanning the bag...';
  const r = await (await fetch('/api/frames?bag=' + i)).json();
  frames = r.frames; saved = new Set(r.saved); k = 0; show();
}
function show() {
  if (!frames.length) return;
  k = Math.max(0, Math.min(frames.length - 1, k));
  const f = frames[k];
  $('img').src = url(k);
  for (const j of [k + 1, k + 2, k + 3]) if (j < frames.length) new Image().src = url(j);
  $('info').textContent = `${k + 1} / ${frames.length}   t = ${f.ts.toFixed(2)} s`;
  $('tags').innerHTML = (saved.has(k) ? tag('SAVED', '#2a2') : '') +
    (f.turning ? tag('turning', '#c60') : '') + (f.blurred ? tag('blurred', '#666') : '');
  $('count').textContent = `${saved.size} saved`;
  drawStrip();
}
function drawStrip() {
  const c = $('strip'), w = c.width = c.clientWidth, n = frames.length, g = c.getContext('2d');
  g.fillStyle = '#333'; g.fillRect(0, 0, w, 24);
  frames.forEach((f, j) => {
    if (f.turning || f.blurred) { g.fillStyle = f.turning ? '#a50' : '#555'; g.fillRect(j * w / n, 8, Math.max(1, w / n), 8); }
  });
  g.fillStyle = '#3d3'; saved.forEach(j => g.fillRect(j * w / n, 0, Math.max(2, w / n), 24));
  g.fillStyle = '#fff'; g.fillRect(k * w / n - 1, 0, 2, 24);
}
async function setSaved(on) {
  const j = k;
  const r = await (await fetch(`/api/save?bag=${bag}&k=${j}&on=${on ? 1 : 0}`, {method: 'POST'})).json();
  if (r.saved) saved.add(j); else saved.delete(j);
  show();
}
function jumpSaved(dir) {
  const s = [...saved].sort((a, b) => a - b);
  const t = dir > 0 ? s.find(j => j > k) : s.reverse().find(j => j < k);
  if (t !== undefined) { k = t; show(); }
}
function stop() { if (timer) { clearInterval(timer); timer = null; } }
$('strip').onclick = e => { k = Math.floor(e.offsetX / $('strip').clientWidth * frames.length); show(); };
window.onresize = drawStrip;
document.onkeydown = e => {
  if (e.target.tagName === 'SELECT') return;
  const step = {ArrowRight: 1, ArrowLeft: -1, ArrowDown: 10, ArrowUp: -10, PageDown: 100, PageUp: -100}[e.key];
  const key = e.key.toLowerCase();
  if (step) { stop(); k += step; show(); }
  else if (e.key === ' ') {
    if (timer) stop();
    else timer = setInterval(() => { if (k >= frames.length - 1) stop(); else { k++; show(); } }, 115);
  }
  else if (key === 's') setSaved(true);
  else if (key === 'u') setSaved(false);
  else if (key === 'n') jumpSaved(1);
  else if (key === 'p') jumpSaved(-1);
  else return;
  e.preventDefault();
};
loadBags();
</script></body></html>
"""


class Bag:
    def __init__(self, path, out):
        self.path = path.rstrip('/')
        self.name = os.path.basename(self.path)
        self.out_dir = os.path.join(out, self.name)
        self.dbs = sorted(glob.glob(os.path.join(self.path, '*.db3')))
        self.frames = None
        self.lock = threading.Lock()

    def load(self, args):
        """Scanned lazily on first use: decodes every image once (~15 s per bag)."""
        with self.lock:
            if self.frames is None:
                t0, images, _, imu = ebf.scan(self.path)
                ts = np.array([i[1] for i in images])
                sharp = np.array([i[2] for i in images])
                steady, ok = ebf.frame_quality(ts, sharp, imu, args)
                self.stamps = [i[0] for i in images]
                self.files = [ebf.frame_name(self.name, t, t0) for t in self.stamps]
                self.frames = [{'ts': float(t), 'turning': bool(not s), 'blurred': bool(s and not o)}
                               for t, s, o in zip(ts, steady, ok)]
        return self.frames

    def saved(self):
        existing = set(os.listdir(self.out_dir)) if os.path.isdir(self.out_dir) else set()
        return [k for k, f in enumerate(self.files) if f in existing]

    def jpeg(self, k):
        for db in self.dbs:
            con = sqlite3.connect(db)
            try:
                row = con.execute(
                    'SELECT m.data FROM messages m JOIN topics t ON m.topic_id = t.id '
                    'WHERE t.name = ? AND m.timestamp = ?', (ebf.IMAGE_TOPIC, self.stamps[k])).fetchone()
            finally:
                con.close()
            if row:
                return bytes(deserialize_message(row[0], CompressedImage).data)
        return None

    def set_saved(self, k, on):
        path = os.path.join(self.out_dir, self.files[k])
        if on and not os.path.exists(path):
            jpeg = self.jpeg(k)
            if jpeg:
                os.makedirs(self.out_dir, exist_ok=True)
                with open(path, 'wb') as f:
                    f.write(jpeg)
        elif not on and os.path.exists(path):
            os.remove(path)
        return os.path.exists(path)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def _send(self, body, ctype='application/json', code=200, cache=False):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if cache:
            self.send_header('Cache-Control', 'max-age=3600')
        self.end_headers()
        self.wfile.write(body)

    def _route(self):
        """(path, bag or None, frame index or None); raises ValueError / KeyError on bad input."""
        u = urlparse(self.path)
        q = parse_qs(u.query)
        bags = self.server.bags
        if 'bag' not in q:
            return u.path, None, None
        b = int(q['bag'][0])
        if not 0 <= b < len(bags):
            raise ValueError
        bag = bags[b]
        bag.load(self.server.args)
        if 'k' not in q:
            return u.path, bag, None
        k = int(q['k'][0])
        if not 0 <= k < len(bag.frames):
            raise ValueError
        return u.path, bag, k

    def do_GET(self):
        try:
            path, bag, k = self._route()
        except (KeyError, ValueError):
            return self._send(b'bad request', 'text/plain', 400)
        if path == '/':
            return self._send(PAGE.encode(), 'text/html; charset=utf-8')
        if path == '/api/bags':
            return self._send(json.dumps([b.name for b in self.server.bags]).encode())
        if path == '/api/frames' and bag:
            return self._send(json.dumps({'frames': bag.frames, 'saved': bag.saved()}).encode())
        if path == '/img' and k is not None:
            jpeg = bag.jpeg(k)
            if jpeg:
                return self._send(jpeg, 'image/jpeg', cache=True)
        self._send(b'not found', 'text/plain', 404)

    def do_POST(self):
        try:
            path, bag, k = self._route()
            on = parse_qs(urlparse(self.path).query)['on'][0] == '1'
        except (KeyError, ValueError):
            return self._send(b'bad request', 'text/plain', 400)
        if path == '/api/save' and k is not None:
            return self._send(json.dumps({'saved': bag.set_saved(k, on)}).encode())
        self._send(b'not found', 'text/plain', 404)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--port', type=int, default=8000)
    ebf.add_quality_args(ap)
    args = ap.parse_args()

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.bags = [Bag(b, args.out) for b in args.bags]
    server.args = args
    print(f'http://localhost:{args.port}  ({len(server.bags)} bags, saving to {args.out}), Ctrl+C to stop')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
