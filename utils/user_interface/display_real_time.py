# pip install fastapi uvicorn pymavlink pyserial pyyaml numpy

import asyncio
import json
import sys
import threading
import time
import logging
from collections import deque
from pathlib import Path

import yaml
import serial.tools.list_ports
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil
import uvicorn

# ── CLI args ──────────────────────────────────────────────────────────────────
# Usage: python display_real_time.py [port] [baud] [calib.yaml]
_DEFAULT_PORT  = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
_DEFAULT_BAUD  = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
_DEFAULT_CALIB = Path(sys.argv[3]) if len(sys.argv) > 3 else \
                 Path(__file__).parent.parent / 'calibration' / 'lab.yaml'

# ── EKF flags ─────────────────────────────────────────────────────────────────
_EKF_ATTITUDE      = 0x0001
_EKF_VEL_HORIZ     = 0x0002
_EKF_POS_HORIZ_REL = 0x0008
_EKF_CONST_POS     = 0x0080

_SIGNAL_TIMEOUT = 3.0
_TRAIL_LEN      = 2400   # 240 s at 10 Hz


# ── quaternion / vector helpers ───────────────────────────────────────────────

def _quat_to_matrix(x, y, z, w):
    return [
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ]

def _col(R, i):   return [R[0][i], R[1][i], R[2][i]]
def _add(a, b):   return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]
def _scale(v, s): return [v[0]*s, v[1]*s, v[2]*s]


# ── lighthouse loading ────────────────────────────────────────────────────────

def _load_lighthouses(yaml_path):
    try:
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
        lhs = []
        for bs_id, geo in cfg.get('geos', {}).items():
            q = geo['rotation_quat']
            lhs.append({'id': str(bs_id), 'pos': geo['position'],
                        'R': _quat_to_matrix(*q)})
        logging.info("Loaded %d lighthouse(s)", len(lhs))
        return lhs
    except Exception as e:
        logging.warning("Could not load lighthouse config: %s", e)
        return []


# ── shared state ──────────────────────────────────────────────────────────────

_lock = threading.Lock()
_state = {
    'x': None, 'y': None, 'z': None,
    'ekf_flags': None, 'ekf_healthy': None,
    'serial_ok': False, 'last_msg': 0.0,
    'port': _DEFAULT_PORT, 'baud': _DEFAULT_BAUD,
    'trail': deque(maxlen=_TRAIL_LEN),
    'reset_token': 0,
}

_stop_event = threading.Event()


# ── MAVLink thread ────────────────────────────────────────────────────────────

def _mavlink_thread(port, baud, stop_event):
    while not stop_event.is_set():
        try:
            mav = mavutil.mavlink_connection(port, baud=baud)
            with _lock:
                _state['serial_ok'] = True
            logging.info("MAVLink connected on %s @ %d", port, baud)
            while not stop_event.is_set():
                msg = mav.recv_match(
                    type=['LOCAL_POSITION_NED', 'EKF_STATUS_REPORT'],
                    blocking=True, timeout=1.0)
                if msg is None:
                    continue
                if msg.get_type() == 'LOCAL_POSITION_NED':
                    now = time.time()
                    x, y, z = msg.x, msg.y, msg.z   # keep NED (z positive-down)
                    with _lock:
                        _state['x'] = x; _state['y'] = y; _state['z'] = z
                        _state['trail'].append((x, y, z, now))
                        _state['last_msg'] = now
                elif msg.get_type() == 'EKF_STATUS_REPORT':
                    flags = msg.flags
                    with _lock:
                        _state['ekf_flags'] = flags
                        _state['ekf_healthy'] = (
                            bool(flags & _EKF_ATTITUDE)
                            and bool(flags & _EKF_POS_HORIZ_REL)
                            and not bool(flags & _EKF_CONST_POS))
                        _state['last_msg'] = time.time()
        except Exception as e:
            with _lock:
                _state['serial_ok'] = False
            logging.warning("MAVLink error (%s), retrying in 2 s...", e)
            stop_event.wait(2.0)


# ── debug thread ──────────────────────────────────────────────────────────────

def _debug_thread(stop_event):
    s, ox, oy, oz = 0.8, 0.1, 0.1, -0.2  # NED: z negative = above origin
    waypoints = [
        (ox,     oy,     oz),
        (ox+s,   oy,     oz),
        (ox+s,   oy+s,   oz),
        (ox,     oy+s,   oz),
        (ox,     oy+s,   oz-s),
        (ox+s,   oy+s,   oz-s),
        (ox+s,   oy,     oz-s),
        (ox,     oy,     oz-s),
    ]
    steps_per_edge, step, edge = 60, 0, 0
    with _lock:
        _state['serial_ok'] = True
    while not stop_event.is_set():
        a = waypoints[edge % len(waypoints)]
        b = waypoints[(edge + 1) % len(waypoints)]
        t = step / steps_per_edge
        x = a[0] + (b[0]-a[0])*t
        y = a[1] + (b[1]-a[1])*t
        z = a[2] + (b[2]-a[2])*t
        now = time.time()
        with _lock:
            _state['x'] = x; _state['y'] = y; _state['z'] = z
            _state['trail'].append((x, y, z, now))
            _state['last_msg']    = now
            _state['ekf_flags']   = 0x000B
            _state['ekf_healthy'] = True
        step += 1
        if step >= steps_per_edge:
            step, edge = 0, (edge + 1) % len(waypoints)
        stop_event.wait(0.1)
    with _lock:
        _state['serial_ok'] = False


# ── connection management ─────────────────────────────────────────────────────

def _start_mavlink(port, baud):
    global _stop_event
    _stop_event.set()
    _stop_event = threading.Event()
    with _lock:
        _state.update(serial_ok=False, last_msg=0.0, port=port, baud=baud)
        _state['trail'].clear()
        _state['reset_token'] = _state.get('reset_token', 0) + 1
    if port == 'debug':
        threading.Thread(target=_debug_thread, args=(_stop_event,), daemon=True).start()
    else:
        threading.Thread(target=_mavlink_thread, args=(port, baud, _stop_event),
                         daemon=True).start()


# ── FastAPI ───────────────────────────────────────────────────────────────────

_lighthouses = _load_lighthouses(_DEFAULT_CALIB)
_STATIC      = Path(__file__).parent.parent.parent / 'docs'


@asynccontextmanager
async def _lifespan(app: FastAPI):
    asyncio.create_task(_broadcast_loop())
    yield

app = FastAPI(lifespan=_lifespan)
app.mount('/static', StaticFiles(directory=str(_STATIC)), name='static')


@app.get('/')
async def index():
    return FileResponse(_STATIC / 'index.html')


@app.get('/api/config')
async def api_config():
    return {
        'lighthouses': [
            {'id': lh['id'], 'pos': lh['pos'], 'R': lh['R']}
            for lh in _lighthouses
        ],
        'default_port': _DEFAULT_PORT,
        'default_baud': _DEFAULT_BAUD,
    }


@app.get('/api/ports')
async def api_ports():
    ports = ['debug'] + [p.device for p in serial.tools.list_ports.comports()]
    return {'ports': ports}


@app.post('/api/connect')
async def api_connect(request: Request):
    body  = await request.json()
    port  = body.get('port', _DEFAULT_PORT)
    baud  = int(body.get('baud', _DEFAULT_BAUD))
    _start_mavlink(port, baud)
    return {'ok': True}


# ── WebSocket ─────────────────────────────────────────────────────────────────

class _WsManager:
    def __init__(self):
        self._clients: list[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self._clients.append(ws)

    def disconnect(self, ws: WebSocket):
        try:
            self._clients.remove(ws)
        except ValueError:
            pass

    async def broadcast(self, text: str):
        dead = []
        for ws in list(self._clients):
            try:
                await ws.send_text(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


_mgr = _WsManager()


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await _mgr.connect(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        _mgr.disconnect(ws)




async def _broadcast_loop():
    last_token = -1
    while True:
        await asyncio.sleep(0.05)   # 20 Hz

        with _lock:
            x           = _state['x']
            y           = _state['y']
            z           = _state['z']
            trail       = _state['trail']
            t           = trail[-1][3] if trail else 0.0
            ekf_flags   = _state['ekf_flags']
            ekf_healthy = _state['ekf_healthy']
            serial_ok   = _state['serial_ok']
            last_msg    = _state['last_msg']
            port        = _state['port']
            baud        = _state['baud']
            token       = _state['reset_token']

        if token != last_token:
            last_token = token
            await _mgr.broadcast(json.dumps({'type': 'reset'}))

        await _mgr.broadcast(json.dumps({
            'type':        'state',
            'x':           x,
            'y':           y,
            'z':           z,
            't':           t,
            'ekf_flags':   ekf_flags,
            'ekf_healthy': ekf_healthy,
            'serial_ok':   serial_ok,
            'last_msg':    last_msg,
            'port':        port,
            'baud':        baud,
        }))


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    # Only auto-connect if a port was explicitly passed as a CLI argument
    if len(sys.argv) > 1:
        _start_mavlink(_DEFAULT_PORT, _DEFAULT_BAUD)
        print(f'Auto-connecting : {_DEFAULT_PORT} @ {_DEFAULT_BAUD} baud')
    else:
        print('No port specified — select one in the browser and click Connect')
    print(f'Calib  : {_DEFAULT_CALIB}')
    print(f'Open   : http://localhost:8050')
    uvicorn.run(app, host='0.0.0.0', port=8050, log_level='warning')
