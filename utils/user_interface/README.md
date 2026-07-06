# `display_real_time.py` — DroneBridge ESP32 connection modes

`display_real_time.py` is the backend for `docs/index.html`. Besides the
serial (USB) path, it can reach the flight controller's MAVLink stream over
WiFi through a DroneBridge ESP32 telemetry radio attached to the FC's telem
port. DroneBridge can be configured two ways, and each needs a different
connection method in the GCS.

## 1. DroneBridge in Access Point (AP) mode

The ESP32 hosts its own WiFi network with a fixed, known IP
(`192.168.4.1` by default). Connect your laptop/phone to that network as a
WiFi client, then use one of:

### 1a. Direct WebSocket — no Python backend needed
DroneBridge serves raw MAVLink over WebSocket at `/mavlink` (not `/`). In
the browser, select the **DB** tab, enter the ESP32's IP/port
(default `192.168.4.1` : `80`), and connect. The browser talks straight to
`ws://<ip>:<port>/mavlink` — `display_real_time.py` isn't involved at all.

### 1b. Via the backend's UDP relay (client mode)
If you'd rather route through the backend (e.g. to also get calibration
config / static file serving from the same server), start
`display_real_time.py`, then in the browser select the **UDP** tab and
enter the ESP32's IP (`192.168.4.1`) with the DroneBridge UDP port
(default `14550`). The backend dials out to that address
(`/ws/udp?host=192.168.4.1&port=14550`) and sends a periodic MAVLink GCS
heartbeat so DroneBridge learns where to forward telemetry.

## 2. DroneBridge in Station / Client mode

The ESP32 joins your existing WiFi network (router) instead of hosting its
own — its IP is then DHCP-assigned and not fixed in advance. Configure
DroneBridge (via its own web UI) to push UDP MAVLink to your GCS machine's
IP on a fixed port (default `14550`).

Start `display_real_time.py`, then in the browser select the **UDP** tab
and **leave the ESP32 IP field blank**, entering only the port. The backend
opens `/ws/udp?port=14550` with no `host`, which makes it listen passively
(`0.0.0.0:<port>`) for DroneBridge's incoming packets instead of dialing
out. The direct WebSocket (DB tab) path doesn't apply here since the ESP's
IP isn't fixed/known ahead of time.

## Summary

| DroneBridge WiFi mode | ESP IP | GCS connection | Backend role |
|---|---|---|---|
| Access Point | fixed (`192.168.4.1`) | **DB** tab → `ws://<ip>:80/mavlink` | none |
| Access Point | fixed (`192.168.4.1`) | **UDP** tab, host = ESP IP | UDP client — dials out, sends GCS heartbeats |
| Station / Client | DHCP, unknown | **UDP** tab, host blank | UDP server — listens passively for DroneBridge's push |

Both UDP sub-modes are handled by the same `/ws/udp` WebSocket endpoint in
`display_real_time.py` — presence/absence of the `host` query param picks
client vs. server behavior.

## Other connection paths

- **Serial (USB)**: browser uses the WebSerial API directly against the
  Pico's USB-CDC port — no backend relay involved, only `/api/ports` and
  `/api/connect` for port discovery/selection.
- **`port=debug`**: runs a synthetic square-wave flight inside
  `display_real_time.py` with no hardware attached, useful for exercising
  the frontend.

See `.claude/rules/utils.md` for running instructions and the GCS
heartbeat/stream-request constraints that apply to all MAVLink paths.
