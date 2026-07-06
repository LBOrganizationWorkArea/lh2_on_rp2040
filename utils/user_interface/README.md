# Connecting the GCS over WiFi (DroneBridge ESP32)

Besides plugging in via USB, the GCS (`docs/index.html`) can talk to the
flight controller over WiFi through a DroneBridge ESP32 radio. There are two
ways to connect, picked with the **DB** / **UDP** tabs in the connection
widget.

## Quick answer: which tab do I use?

- **Know the ESP32's IP address?** → Use the **UDP** tab. Type the IP in,
  run `python display_real_time.py`, hit Connect. This works whether
  DroneBridge is hosting its own WiFi (Access Point, IP is always
  `192.168.4.1`) or has joined your home/lab WiFi (Client mode — find its IP
  in your router's device list or DroneBridge's own status page).
- **Don't want to run the Python backend at all?** → Use the **DB** tab
  instead. It connects the browser straight to the ESP32
  (`ws://<ip>:80/mavlink`), no backend needed. Only works when you're on the
  same network as the ESP32 (typically its own Access Point).

That's the 90% case. Just enter the IP and connect.

## The one exception

Leave the IP field blank in the **UDP** tab only if you've configured
DroneBridge (in its own web UI) to actively push telemetry to your computer,
instead of waiting for the GCS to connect to it. In that setup the backend
just listens for incoming packets rather than dialing out.

## Other ways to connect

- **USB serial**: plug in the Pico, pick the port in the **SERIAL** tab —
  no WiFi or backend relay involved.
- **Debug mode**: pick `debug` as the port to see a simulated flight with no
  hardware attached at all.

Run the backend from `utils/user_interface/`:
```bash
python display_real_time.py
```
See `.claude/rules/utils.md` for more on running it and MAVLink stream
requirements.
