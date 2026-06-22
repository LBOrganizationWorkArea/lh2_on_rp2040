# Website / GCS Frontend Rules (`docs/`)

## Overview

`docs/index.html` is a **single-file** browser GCS. It has no build step, no bundler, no dependencies — everything is inline HTML/CSS/JS.

It requires `utils/user_interface/display_real_time.py` to be running as a backend. It connects to that server via WebSocket on the same host/port.

## Editing

Edit `docs/index.html` directly. Because there is no build step, changes are immediately reflected on reload.

Key sections in the file (navigate by searching):

| Section | What it does |
|---|---|
| `#panel` | Left sidebar: connection, EKF flags, position readout, artificial horizon |
| `#map` | Top-down 2D position plot (canvas) |
| WebSocket handler (`ws.onmessage`) | Receives state JSON from backend, updates all widgets |
| `api_connect` / `api_ports` | REST calls to `display_real_time.py` to open serial port |

## State format

The backend pushes JSON over WebSocket. Key fields:
- `pos`: `[x, y, z]` in metres (world frame)
- `quat`: `[x, y, z, w]` attitude quaternion
- `ekf_flags`: integer bitmask from EKF_STATUS_REPORT
- `serial_ok`: bool

## No framework

Do not introduce React, Vue, or any npm toolchain. Keep it a single HTML file.
