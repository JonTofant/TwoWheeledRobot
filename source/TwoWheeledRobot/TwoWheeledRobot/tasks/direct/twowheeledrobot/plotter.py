"""
Interactive diagnostic plotter — served at http://localhost:8988

Uses Plotly.js (loaded from CDN in the browser) so there are no extra
Python dependencies.  Features: zoom, pan, scroll, pause, real time axis.

Architecture
------------
  Isaac Sim process
    push() thread  →  writes /tmp/robot_debug_data.json  every update cycle

  Separate OS process (not a thread — bypasses Isaac Sim GIL issues)
    HTTP server    →  serves the HTML page + /data.json endpoint
"""

from __future__ import annotations

import collections
import json
import multiprocessing
import threading
import time

_DATA_PATH = "/tmp/robot_debug_data.json"
_HTTP_PORT  = 8988

# ── HTML page (served once, then JS polls /data.json) ─────────────────────────

_HTML = """\
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Robot diagnostics</title>
<script src="https://cdn.plot.ly/plotly-2.32.0.min.js"></script>
<style>
  body  { margin:0; background:#1a1a2e; color:#eee; font-family:sans-serif; }
  #ctrl { padding:8px 12px; display:flex; gap:12px; align-items:center;
          background:#16213e; border-bottom:1px solid #0f3460; }
  button{ background:#0f3460; color:#eee; border:1px solid #e94560;
          padding:4px 14px; border-radius:4px; cursor:pointer; font-size:13px; }
  button:hover { background:#e94560; }
  #status { font-size:12px; color:#aaa; margin-left:auto; }
  #plot { width:100vw; }
</style>
</head>
<body>
<div id="ctrl">
  <b>Two-Wheeled Robot &mdash; diagnostics</b>
  <button id="btn" onclick="togglePause()">&#9646;&#9646; Pause</button>
  <button onclick="Plotly.relayout('plot',{'xaxis.autorange':true,'xaxis2.autorange':true,'xaxis3.autorange':true,'xaxis4.autorange':true,'xaxis5.autorange':true,'yaxis.autorange':true,'yaxis2.autorange':true,'yaxis3.autorange':true,'yaxis4.autorange':true,'yaxis5.autorange':true})">&#8635; Reset zoom</button>
  <span id="status">connecting&hellip;</span>
</div>
<div id="plot"></div>

<script>
var paused = false;
var initialized = false;

function togglePause() {
  paused = !paused;
  document.getElementById('btn').innerHTML = paused ? '&#9654; Resume' : '&#9646;&#9646; Pause';
}

var layout = {
  grid: { rows:5, columns:1, pattern:'independent', roworder:'top to bottom' },
  height: window.innerHeight - 48,
  paper_bgcolor: '#1a1a2e',
  plot_bgcolor:  '#16213e',
  font:  { color:'#ccc', size:11 },
  margin:{ l:60, r:20, t:10, b:40 },
  showlegend: true,
  legend: { bgcolor:'rgba(0,0,0,0)', font:{ size:10 } },

  xaxis:  { title:'', color:'#aaa', gridcolor:'#2a2a4a', zeroline:false },
  xaxis2: { title:'', color:'#aaa', gridcolor:'#2a2a4a', zeroline:false },
  xaxis3: { title:'', color:'#aaa', gridcolor:'#2a2a4a', zeroline:false },
  xaxis4: { title:'', color:'#aaa', gridcolor:'#2a2a4a', zeroline:false },
  xaxis5: { title:'Time (s)', color:'#aaa', gridcolor:'#2a2a4a', zeroline:false },

  yaxis:  { title:'Tilt (deg)',     color:'#aaa', gridcolor:'#2a2a4a', zeroline:true, zerolinecolor:'#555' },
  yaxis2: { title:'Tilt rate (r/s)',color:'#aaa', gridcolor:'#2a2a4a', zeroline:true, zerolinecolor:'#555' },
  yaxis3: { title:'Yaw rate (r/s)', color:'#aaa', gridcolor:'#2a2a4a', zeroline:true, zerolinecolor:'#555' },
  yaxis4: { title:'Torque (Nm)',    color:'#aaa', gridcolor:'#2a2a4a', zeroline:true, zerolinecolor:'#555' },
  yaxis5: { title:'Velocity (m/s)',  color:'#aaa', gridcolor:'#2a2a4a', zeroline:true, zerolinecolor:'#555' },
};

var traces = [
  { name:'tilt',      xaxis:'x',  yaxis:'y',  line:{color:'#4fc3f7',width:1.5} },
  { name:'θ_des',     xaxis:'x',  yaxis:'y',  line:{color:'#ff8f00',width:1.2,dash:'dot'} },
  { name:'tilt rate', xaxis:'x2', yaxis:'y2', line:{color:'#81c784',width:1.5} },
  { name:'yaw rate',  xaxis:'x3', yaxis:'y3', line:{color:'#ce93d8',width:1.5} },
  { name:'τ_L',       xaxis:'x4', yaxis:'y4', line:{color:'#ef5350',width:1.2} },
  { name:'τ_R',       xaxis:'x4', yaxis:'y4', line:{color:'#ab47bc',width:1.2} },
  { name:'vel_cmd',   xaxis:'x5', yaxis:'y5', line:{color:'#ffb74d',width:1.2,dash:'dot'} },
  { name:'vel_fwd',   xaxis:'x5', yaxis:'y5', line:{color:'#26c6da',width:1.5} },
];
traces.forEach(function(t){ t.type='scatter'; t.mode='lines'; t.x=[]; t.y=[]; });

Plotly.newPlot('plot', traces, layout, {
  responsive:true, scrollZoom:true,
  modeBarButtonsToRemove:['select2d','lasso2d','autoScale2d'],
  displaylogo:false
});

var keys = ['tilt','theta_des','tilt_rate','yaw_rate','torque_L','torque_R','vel_cmd','vel_fwd'];

function update() {
  if (paused) return;
  fetch('/data.json?t=' + Date.now())
    .then(function(r){ return r.json(); })
    .then(function(d) {
      var t = d.t;
      var upd = {
        x: [ t,t, t, t, t,t, t,t ],
        y: keys.map(function(k){ return d[k] || []; })
      };
      var idx = traces.map(function(_,i){ return i; });
      Plotly.restyle('plot', upd, idx);
      document.getElementById('status').textContent =
        'step ' + (d.step||0) + '  |  t = ' + (t[t.length-1]||0).toFixed(2) + ' s';
    })
    .catch(function(){ document.getElementById('status').textContent = 'no data yet'; });
}

setInterval(update, 200);
window.addEventListener('resize', function(){
  Plotly.relayout('plot', {height: window.innerHeight - 48});
});
</script>
</body>
</html>
""".encode("utf-8")


# ── Plotter class ──────────────────────────────────────────────────────────────

class RealtimePlotter:

    def __init__(
        self,
        enabled:    bool  = True,
        window_sec: float = 10.0,
        dt:         float = 0.02,
        update_hz:  float = 10.0,
    ) -> None:
        self.enabled = enabled
        if not enabled:
            return

        self._dt         = dt
        self._window_pts = max(1, int(window_sec / dt))
        self._update_int = 1.0 / update_hz
        self._step       = 0
        self._running    = True

        _n = self._window_pts
        self._bufs: dict[str, collections.deque] = {
            "tilt":      collections.deque([0.0] * _n, maxlen=_n),
            "tilt_rate": collections.deque([0.0] * _n, maxlen=_n),
            "theta_des": collections.deque([0.0] * _n, maxlen=_n),
            "torque_L":  collections.deque([0.0] * _n, maxlen=_n),
            "torque_R":  collections.deque([0.0] * _n, maxlen=_n),
            "vel_cmd":   collections.deque([0.0] * _n, maxlen=_n),
            "vel_fwd":   collections.deque([0.0] * _n, maxlen=_n),
            "yaw_rate":  collections.deque([0.0] * _n, maxlen=_n),
        }

        # Write initial empty JSON so the server has something to serve
        self._flush()

        # HTTP server in a separate OS process (immune to Isaac Sim GIL)
        multiprocessing.Process(
            target=_http_server_process,
            args=(_DATA_PATH, _HTTP_PORT),
            daemon=True,
        ).start()

        # Data writer in a background thread
        threading.Thread(
            target=self._data_loop,
            name="PlotterData",
            daemon=True,
        ).start()

    # ── Public API ─────────────────────────────────────────────────────────────

    def push(self, data: dict) -> None:
        if not self.enabled:
            return
        for key, buf in self._bufs.items():
            if key in data:
                buf.append(float(data[key]))
        self._step += 1

    def close(self) -> None:
        if self.enabled:
            self._running = False

    # ── Internal ───────────────────────────────────────────────────────────────

    def _flush(self) -> None:
        n = self._window_pts
        t = [round(-n * self._dt + i * self._dt, 4) for i in range(n)]
        payload = {"t": t, "step": self._step}
        payload.update({k: list(v) for k, v in self._bufs.items()})
        tmp = _DATA_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f)
        import os
        os.replace(tmp, _DATA_PATH)   # atomic on Linux

    def _data_loop(self) -> None:
        last = time.monotonic()
        while self._running:
            now = time.monotonic()
            if now - last >= self._update_int:
                last = now
                try:
                    self._flush()
                except Exception:
                    pass
            time.sleep(0.01)


# ── HTTP server (runs in a separate process) ───────────────────────────────────

def _kill_port(port: int) -> None:
    import os, signal
    hex_port = f"{port:04X}"
    try:
        for tcp_file in ("/proc/net/tcp", "/proc/net/tcp6"):
            try:
                lines = open(tcp_file).readlines()
            except FileNotFoundError:
                continue
            for line in lines[1:]:
                parts = line.split()
                if len(parts) < 10:
                    continue
                if parts[1].endswith(f":{hex_port}") and parts[3] == "0A":
                    inode = int(parts[9])
                    for pid in os.listdir("/proc"):
                        if not pid.isdigit():
                            continue
                        try:
                            for fd in os.listdir(f"/proc/{pid}/fd"):
                                try:
                                    if os.readlink(f"/proc/{pid}/fd/{fd}") == f"socket:[{inode}]":
                                        os.kill(int(pid), signal.SIGTERM)
                                except OSError:
                                    pass
                        except PermissionError:
                            pass
    except Exception:
        pass


def _http_server_process(data_path: str, port: int) -> None:
    import socket, time as _time

    _kill_port(port)
    _time.sleep(0.3)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass
    try:
        srv.bind(("0.0.0.0", port))
    except OSError as e:
        print(f"[Plotter] Cannot bind :{port}: {e}")
        return
    srv.listen(8)
    print(f"[Plotter] http://localhost:{port}")

    while True:
        try:
            conn, _ = srv.accept()
        except Exception:
            continue
        threading.Thread(
            target=_serve_conn,
            args=(conn, data_path),
            daemon=True,
        ).start()


def _serve_conn(conn, data_path: str) -> None:
    try:
        conn.settimeout(3.0)
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = conn.recv(2048)
            if not chunk:
                break
            buf += chunk
            if len(buf) > 32768:
                break

        line  = buf.split(b"\r\n")[0].decode(errors="replace")
        parts = line.split()
        path  = parts[1].split("?")[0] if len(parts) >= 2 else "/"

        if path == "/":
            _respond(conn, 200, "text/html; charset=utf-8", _HTML)
        elif path == "/data.json":
            try:
                with open(data_path, "rb") as f:
                    body = f.read()
                _respond(conn, 200, "application/json", body,
                         extra="Cache-Control: no-store\r\n")
            except FileNotFoundError:
                _respond(conn, 200, "application/json", b"{}")
        else:
            _respond(conn, 404, "text/plain", b"not found")
    except Exception:
        pass
    finally:
        try:
            conn.close()
        except Exception:
            pass


def _respond(conn, code: int, ctype: str, body: bytes, extra: str = "") -> None:
    status = {200: "OK", 404: "Not Found"}.get(code, "")
    header = (
        f"HTTP/1.1 {code} {status}\r\n"
        f"Content-Type: {ctype}\r\n"
        f"Content-Length: {len(body)}\r\n"
        f"Connection: close\r\n"
        f"{extra}"
        f"\r\n"
    ).encode()
    conn.sendall(header + body)
