# Lighthouse 2.0 factory calibration

This project needs the factory calibration parameters for each Lighthouse 2.0 base station before applying accurate angle corrections:

- phase
- tilt
- gibmag / gib magnitude
- gibphase / gib phase
- curve
- ogeemag / ogee magnitude
- ogeephase / ogee phase

These values are unique to each Lighthouse. Do not replace them with generic values.

## Connect one Lighthouse

1. Power one Lighthouse 2.0 with its normal power adapter.
2. Connect only that Lighthouse to the PC with USB.
3. If the RP2040 is already on `COM3`, unplug the RP2040 or use the COM port that belongs to the Lighthouse.
4. Read each Lighthouse separately, then repeat for the next one.

The Bitcraze Lighthouse setup documentation uses this same basic USB workflow for Lighthouse V2 base-station configuration: power one base station, connect it over micro-USB, scan/read it, disconnect it, then repeat for the other base station.

## Find the COM port on Windows

From PowerShell in `C:\Users\elkah\lh2_positioning\positioningv3`:

```powershell
py .\tools\read_lh2_factory_calibration.py --list-ports
```

You can also open Windows Device Manager and check **Ports (COM & LPT)**.

If `COM3` is already used by the RP2040, disconnect the RP2040 while reading the Lighthouse calibration, or use the other COM port shown by `--list-ports`.

## Install dependencies

The project already lists `pyserial` in `requirements.txt`.

```powershell
py -m pip install -r requirements.txt
```

Direct Lighthouse 2.0 factory calibration reading uses the Bitcraze/Valve USB protocol. This project wrapper can call Bitcraze's `get_lh2_calib_data.py` when you have it locally.

Typical setup:

```powershell
cd C:\Users\elkah
git clone https://github.com/bitcraze/crazyflie-firmware.git
```

If you do not want to clone the repository locally, you can also copy/paste Bitcraze's Lighthouse reader into this project. The wrapper auto-detects these paths:

```text
tools\get_lh2_calib_data.py
tools\bitcraze\get_lh2_calib_data.py
third_party\bitcraze\crazyflie-firmware\tools\lighthouse\get_lh2_calib_data.py
vendor\bitcraze\crazyflie-firmware\tools\lighthouse\get_lh2_calib_data.py
external\bitcraze\crazyflie-firmware\tools\lighthouse\get_lh2_calib_data.py
```

If you copy source from Bitcraze, keep its license header and attribution with the copied files.

This project currently includes the old Bitcraze reader at:

```text
tools\bitcraze\get_lh2_calib_data.py
```

It was copied from a public fork of `crazyflie-firmware` that still has the historical Bitcraze file:

```text
https://github.com/PKU-MACDLab/MACDLab_Exp/blob/d9ead3fb5e57d5d62208341729d144c63ae1383d/crazyflie-firmware/tools/lighthouse/get_lh2_calib_data.py
```

Then either pass the script path:

```powershell
py .\tools\read_lh2_factory_calibration.py --dev COM3 --output .\config\lighthouse_factory_calibration_bs4.json --bitcraze-script C:\Users\elkah\crazyflie-firmware\tools\lighthouse\get_lh2_calib_data.py
```

Or set an environment variable for the current PowerShell session:

```powershell
$env:BITCRAZE_LH2_CALIB_SCRIPT="C:\Users\elkah\crazyflie-firmware\tools\lighthouse\get_lh2_calib_data.py"
```

## Commands

List ports:

```powershell
py .\tools\read_lh2_factory_calibration.py --list-ports
```

Read BS4:

```powershell
py .\tools\read_lh2_factory_calibration.py --dev COM3 --output .\config\lighthouse_factory_calibration_bs4.json
```

Read BS10:

```powershell
py .\tools\read_lh2_factory_calibration.py --dev COM4 --output .\config\lighthouse_factory_calibration_bs10.json
```

Save using the Lighthouse serial number when it is available:

```powershell
py .\tools\read_lh2_factory_calibration.py --dev COM3 --output-dir .\config
```

This creates:

```text
config\lighthouse_factory_calibration_<serial>.json
```

If the serial number is not reported by the Bitcraze reader, the script falls back to the device name, for example:

```text
config\lighthouse_factory_calibration_COM3.json
```

## Expected JSON

The script saves this project schema:

```json
{
  "source": "usb",
  "device": "COM3",
  "timestamp": "2026-05-14T12:00:00+00:00",
  "base_station": {
    "serial": "F4A1A908",
    "channel": null,
    "model": "LH2"
  },
  "calibration": {
    "axis0": {
      "phase": 0.0,
      "tilt": 0.0,
      "gibmag": 0.0,
      "gibphase": 0.0,
      "curve": 0.0,
      "ogeemag": 0.0,
      "ogeephase": 0.0
    },
    "axis1": {
      "phase": 0.0,
      "tilt": 0.0,
      "gibmag": 0.0,
      "gibphase": 0.0,
      "curve": 0.0,
      "ogeemag": 0.0,
      "ogeephase": 0.0
    }
  }
}
```

The numbers above are only an example of the file shape. Real values must come from the Lighthouse.

## Load from Python

Use the reusable loader:

```python
from tools.lighthouse_factory_calibration import load_lighthouse_factory_calibration

factory = load_lighthouse_factory_calibration(
    "config/lighthouse_factory_calibration_bs4.json"
)
axis0 = factory["calibration"]["axis0"]
axis1 = factory["calibration"]["axis1"]
```

The loader validates that both axes contain all required numeric fields.

## Use in geometry calibration

The default geometry estimator now auto-loads these two files when they exist:

```text
config\lighthouse_factory_calibration_bs4.json
config\lighthouse_factory_calibration_bs10.json
```

Run:

```powershell
py .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py
```

Or pass the files explicitly:

```powershell
py .\tools\04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py --factory-calibs "4=.\config\lighthouse_factory_calibration_bs4.json,10=.\config\lighthouse_factory_calibration_bs10.json"
```

The saved geometry file includes the factory parameters used for each base station. `05_live_position.py` then reads them from the geometry file automatically.

The implemented LH2 factory measurement model uses `phase`, `tilt`, `gibmag`, and `gibphase`. The `curve`, `ogeemag`, and `ogeephase` values are preserved in JSON and in the geometry output, but are not applied by the current Bitcraze-style subset model.

## Troubleshooting

- Wrong COM port: run `--list-ports`, unplug the RP2040 if needed, and connect only one Lighthouse.
- Lighthouse not detected: check power, USB cable, and Device Manager.
- Port already open: close serial monitors, Python scripts, Arduino tools, and anything using the same COM port.
- Missing dependency: run `py -m pip install -r requirements.txt`.
- Protocol reader missing: clone Bitcraze `crazyflie-firmware` and point `--bitcraze-script` to `tools\lighthouse\get_lh2_calib_data.py`.
- USB protocol not implemented: this project wrapper does not guess or synthesize factory calibration. It only saves data returned by a compatible Bitcraze reader.
- Permission error: close other programs using the COM port, reconnect the USB cable, or try another USB port.
