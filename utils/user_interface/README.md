# Connecting the GCS over WiFi (DroneBridge ESP32)

Besides plugging in via USB, the GCS (`docs/index.html`) can talk to the
flight controller over WiFi through a DroneBridge ESP32 radio. How you
connect depends on which WiFi mode DroneBridge is set to.

## Access Point mode (ESP32 hosts its own WiFi)

Connect your laptop/phone to the ESP32's own network. Its IP is always
`192.168.4.1`. Then either:

- **DB tab** — no backend needed. Connects the browser straight to the ESP32
  (`ws://192.168.4.1:80/mavlink`). Simplest option.
- **UDP tab** — run `python display_real_time.py`, then enter `192.168.4.1`
  as the IP and connect. Use this if you want the backend running anyway
  (e.g. for calibration config).

## Client mode (ESP32 joins your WiFi)

The ESP32's IP is assigned by your router (DHCP), so find it first — check
your router's device list or DroneBridge's own status page.

- **UDP tab** — run `python display_real_time.py`, enter the ESP32's IP,
  and connect.

The **DB tab** also works here if DroneBridge exposes `/mavlink` in Client
mode — just point it at the discovered IP instead of `192.168.4.1`.

## Exception: DroneBridge pushes to you instead

If you've configured DroneBridge (in its own web UI) to actively send
telemetry to your computer's address, leave the IP field blank in the
**UDP** tab. The backend then just listens for incoming packets instead of
connecting out.

## Other ways to connect

- **USB serial**: plug in the Pico, pick the port in the **SERIAL** tab —
  no WiFi or backend relay involved.
- **Debug mode**: pick `debug` as the port to see a simulated flight with no
  hardware attached at all.

See `.claude/rules/utils.md` for more on running the backend and MAVLink
stream requirements.
