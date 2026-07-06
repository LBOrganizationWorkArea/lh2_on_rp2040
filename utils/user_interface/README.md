# `display_real_time.py` — DroneBridge ESP32 connection modes

`display_real_time.py` is the backend for `docs/index.html`. Besides the
serial (USB) path, it can reach the flight controller's MAVLink stream over
WiFi through a DroneBridge ESP32 telemetry radio attached to the FC's telem
port.

The key variable is **whether you know the ESP32's IP address**, not which
WiFi mode DroneBridge itself is in — that's true whether DroneBridge is
hosting its own network (Access Point) or has joined yours (Client/Station).

## 1. DroneBridge in Access Point (AP) mode

The ESP32 hosts its own WiFi network with a fixed, known IP
(`192.168.4.1` by default). Connect your laptop/phone to that network as a
WiFi client, then use one of:

### 1a. Direct WebSocket — no Python backend needed
DroneBridge serves raw MAVLink over WebSocket at `/mavlink` (not `/`). In
the browser, select the **DB** tab, enter the ESP32's IP/port
(default `192.168.4.1` : `80`), and connect. The browser talks straight to
`ws://<ip>:<port>/mavlink` — `display_real_time.py` isn't involved at all.

### 1b. Via the backend's UDP relay (client/dial-out mode)
If you'd rather route through the backend (e.g. to also get calibration
config / static file serving from the same server), start
`display_real_time.py`, then in the browser select the **UDP** tab and
enter the ESP32's IP (`192.168.4.1`) with the DroneBridge UDP port
(default `14550`). The backend dials out to that address
(`/ws/udp?host=192.168.4.1&port=14550`) and sends a periodic MAVLink GCS
heartbeat so DroneBridge learns where to forward telemetry.

## 2. DroneBridge in Station / Client mode

The ESP32 joins your existing WiFi network (router) instead of hosting its
own, so its IP is DHCP-assigned rather than fixed. Find that IP (router's
DHCP client list, or DroneBridge's own status page/serial output), then use
the **UDP** tab exactly as in 1b: enter the discovered IP and the DroneBridge
UDP port (default `14550`). The backend dials out to
`/ws/udp?host=<esp-ip>&port=14550` and sends GCS heartbeats so DroneBridge
starts forwarding — this is the normal way to use the UDP tab in practice.

The direct WebSocket (**DB** tab) path also works here if DroneBridge exposes
`/mavlink` in Client mode too — just point it at the discovered IP instead of
`192.168.4.1`.

### When to leave the ESP IP field blank instead

If DroneBridge is configured (via its own web UI) to actively push UDP
MAVLink out to a fixed target IP:port — i.e. *it* dials out to *you* instead
of waiting to be dialed — leave the ESP IP field blank in the **UDP** tab.
The backend then opens `/ws/udp?port=14550` with no `host`, listening
passively on `0.0.0.0:<port>` for DroneBridge's incoming packets. This is
the exception, not the default; most setups use the dial-out flow above.

## Summary

| Scenario | ESP IP | GCS connection | Backend role |
|---|---|---|---|
| Access Point | fixed (`192.168.4.1`) | **DB** tab → `ws://<ip>:80/mavlink` | none |
| Access Point | fixed (`192.168.4.1`) | **UDP** tab, host = ESP IP | UDP client — dials out, sends GCS heartbeats |
| Station / Client | DHCP, discovered | **UDP** tab, host = discovered ESP IP | UDP client — dials out, sends GCS heartbeats |
| Either (DroneBridge configured to push to GCS) | n/a | **UDP** tab, host blank | UDP server — listens passively for DroneBridge's push |

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
