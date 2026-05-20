#!/usr/bin/env python3

import argparse
import ast
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from lighthouse_factory_calibration import (
    CALIBRATION_FIELDS,
    normalize_lighthouse_factory_calibration,
)


NUMBER_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


class CalibrationReadError(RuntimeError):
    pass


def list_serial_ports():
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise CalibrationReadError(
            "Missing dependency: pyserial. Install it with: py -m pip install pyserial"
        ) from exc

    return list(list_ports.comports())


def print_serial_ports():
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        details = []
        if port.description:
            details.append(port.description)
        if port.vid is not None and port.pid is not None:
            details.append(f"VID:PID={port.vid:04X}:{port.pid:04X}")
        if port.serial_number:
            details.append(f"serial={port.serial_number}")

        suffix = f" ({', '.join(details)})" if details else ""
        print(f"{port.device}{suffix}")


def find_default_bitcraze_script():
    candidates = []

    env_path = os.environ.get("BITCRAZE_LH2_CALIB_SCRIPT")
    if env_path:
        candidates.append(Path(env_path))

    cwd = Path.cwd()
    candidates.extend([
        cwd / "get_lh2_calib_data.py",
        cwd / "tools" / "get_lh2_calib_data.py",
        cwd / "tools" / "bitcraze" / "get_lh2_calib_data.py",
        cwd / "third_party" / "bitcraze" / "crazyflie-firmware" / "tools" / "lighthouse" / "get_lh2_calib_data.py",
        cwd / "vendor" / "bitcraze" / "crazyflie-firmware" / "tools" / "lighthouse" / "get_lh2_calib_data.py",
        cwd / "external" / "bitcraze" / "crazyflie-firmware" / "tools" / "lighthouse" / "get_lh2_calib_data.py",
        cwd.parent / "crazyflie-firmware" / "tools" / "lighthouse" / "get_lh2_calib_data.py",
        Path.home() / "crazyflie-firmware" / "tools" / "lighthouse" / "get_lh2_calib_data.py",
        Path.home() / "projects" / "crazyflie-firmware" / "tools" / "lighthouse" / "get_lh2_calib_data.py",
    ])

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    return None


def read_text_with_bitcraze_script(dev, script_path, timeout_s):
    cmd = [sys.executable, str(script_path), "--dev", dev]

    try:
        completed = subprocess.run(
            cmd,
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
    except PermissionError as exc:
        raise CalibrationReadError(
            f"Permission denied while running Bitcraze script: {script_path}"
        ) from exc
    except subprocess.TimeoutExpired as exc:
        raise CalibrationReadError(
            f"Timed out after {timeout_s}s while reading {dev}. Check the COM port and USB cable."
        ) from exc

    text = "\n".join(part for part in (completed.stdout, completed.stderr) if part)
    if completed.returncode != 0:
        raise CalibrationReadError(
            "Bitcraze calibration script failed.\n"
            f"Command: {' '.join(cmd)}\n"
            f"Output:\n{text.strip() or '(no output)'}"
        )

    if not text.strip():
        raise CalibrationReadError(
            "Bitcraze calibration script completed but returned no calibration data."
        )

    return text


def find_json_object(text):
    decoder = json.JSONDecoder()
    for match in re.finditer(r"\{", text):
        try:
            obj, _ = decoder.raw_decode(text[match.start():])
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            return obj
    return None


def find_python_object(text):
    for match in re.finditer(r"\{", text):
        snippet = text[match.start():]
        for end_match in reversed(list(re.finditer(r"\}", snippet))):
            candidate = snippet[:end_match.end()]
            try:
                obj = ast.literal_eval(candidate)
            except (ValueError, SyntaxError):
                continue
            if isinstance(obj, dict):
                return obj
    return None


def parse_number_list(value):
    value = value.strip().rstrip(",")
    value = value.replace("{", "[").replace("}", "]")

    try:
        parsed = ast.literal_eval(value)
    except (ValueError, SyntaxError):
        parsed = None

    if isinstance(parsed, (list, tuple)) and len(parsed) >= 2:
        return [float(parsed[0]), float(parsed[1])]

    numbers = re.findall(NUMBER_RE, value)
    if len(numbers) >= 2:
        return [float(numbers[0]), float(numbers[1])]

    return None


def parse_bitcraze_text(text):
    json_obj = find_json_object(text)
    if json_obj is not None:
        return coerce_to_project_schema(json_obj)

    python_obj = find_python_object(text)
    if python_obj is not None:
        return coerce_to_project_schema(python_obj)

    assignment_data = parse_bitcraze_assignment_output(text)
    if assignment_data is not None:
        return assignment_data

    pairs = {}
    for field in CALIBRATION_FIELDS:
        pattern = rf"(?im)^\s*(?:\.|\"|')?{re.escape(field)}(?:\"|')?\s*[:=]\s*(.+?)\s*$"
        match = re.search(pattern, text)
        if match:
            values = parse_number_list(match.group(1))
            if values is not None:
                pairs[field] = values

    if not all(field in pairs for field in CALIBRATION_FIELDS):
        missing = [field for field in CALIBRATION_FIELDS if field not in pairs]
        raise CalibrationReadError(
            "Could not parse factory calibration values from Bitcraze output. "
            f"Missing fields: {', '.join(missing)}"
        )

    serial = parse_serial(text)
    channel = parse_channel(text)

    return {
        "base_station": {
            "serial": serial,
            "channel": channel,
            "model": "LH2",
        },
        "calibration": {
            "axis0": {field: pairs[field][0] for field in CALIBRATION_FIELDS},
            "axis1": {field: pairs[field][1] for field in CALIBRATION_FIELDS},
        },
    }


def parse_bitcraze_assignment_output(text):
    axes = {"axis0": {}, "axis1": {}}

    patterns = [
        rf"(?im)^\s*calib\.sweeps\[(0|1)\]\.({'|'.join(CALIBRATION_FIELDS)})\s*=\s*({NUMBER_RE})\s*$",
        rf"(?im)^\s*fcal\.(0|1)\.({'|'.join(CALIBRATION_FIELDS)})\s+({NUMBER_RE})\s*$",
    ]

    for pattern in patterns:
        for match in re.finditer(pattern, text):
            axis_name = f"axis{match.group(1)}"
            field = match.group(2)
            value = float(match.group(3))
            axes[axis_name][field] = value

    found_any = any(axes[axis_name] for axis_name in axes)
    if not found_any:
        return None

    missing = []
    for axis_name in ("axis0", "axis1"):
        for field in CALIBRATION_FIELDS:
            if field not in axes[axis_name]:
                missing.append(f"{axis_name}.{field}")

    if missing:
        raise CalibrationReadError(
            "Parsed Bitcraze output, but it did not contain a complete factory calibration. "
            f"Missing fields: {', '.join(missing)}"
        )

    serial = parse_serial(text)
    channel = parse_channel(text)

    return {
        "base_station": {
            "serial": serial,
            "channel": channel,
            "model": "LH2",
        },
        "calibration": axes,
    }


def parse_serial(text):
    patterns = [
        r"(?i)\bserial(?:\s+number)?\s*[:=]\s*([A-Za-z0-9_-]+)",
        r"(?i)\bbase\s*station\s*([A-F0-9]{6,})\b",
        r"(?i)\bgot\s+calibration\s+from\s+([A-F0-9]{6,})\b",
    ]
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return match.group(1)
    return None


def parse_channel(text):
    match = re.search(r"(?i)\bchannel\s*[:=]?\s*(\d+)\b", text)
    if match:
        return int(match.group(1))
    return None


def coerce_to_project_schema(data):
    if "calibration" in data:
        return data

    source = data
    for key in ("base_station", "basestation", "bs", "calib", "calibration_data"):
        if isinstance(source, dict) and isinstance(source.get(key), dict):
            nested = source[key]
            if "calibration" in nested:
                return nested

    axes = {}
    for axis_name in ("axis0", "axis1"):
        if isinstance(data.get(axis_name), dict):
            axes[axis_name] = data[axis_name]

    if len(axes) == 2:
        return {
            "base_station": {
                "serial": data.get("serial"),
                "channel": data.get("channel"),
                "model": "LH2",
            },
            "calibration": axes,
        }

    pairs = {}
    for field in CALIBRATION_FIELDS:
        value = data.get(field)
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            pairs[field] = [float(value[0]), float(value[1])]

    if all(field in pairs for field in CALIBRATION_FIELDS):
        return {
            "base_station": {
                "serial": data.get("serial"),
                "channel": data.get("channel"),
                "model": "LH2",
            },
            "calibration": {
                "axis0": {field: pairs[field][0] for field in CALIBRATION_FIELDS},
                "axis1": {field: pairs[field][1] for field in CALIBRATION_FIELDS},
            },
        }

    raise CalibrationReadError(
        "The Bitcraze output was JSON-like, but it did not contain the expected "
        "LH2 factory calibration fields."
    )


def read_factory_calibration(dev, bitcraze_script=None, timeout_s=20):
    ports = list_serial_ports()
    known_ports = {port.device.upper() for port in ports}
    if dev.upper() not in known_ports:
        available = ", ".join(port.device for port in ports) or "none"
        raise CalibrationReadError(
            f"COM port not found: {dev}. Available serial ports: {available}"
        )

    script_path = Path(bitcraze_script) if bitcraze_script else find_default_bitcraze_script()
    if script_path is None:
        raise CalibrationReadError(
            "USB factory calibration read is not implemented natively in this project yet. "
            "Install or clone Bitcraze crazyflie-firmware and point this tool to "
            "tools/lighthouse/get_lh2_calib_data.py with --bitcraze-script, or set "
            "BITCRAZE_LH2_CALIB_SCRIPT. This tool will not invent calibration values."
        )

    if not script_path.is_file():
        raise CalibrationReadError(f"Bitcraze script not found: {script_path}")

    text = read_text_with_bitcraze_script(dev, script_path, timeout_s)
    parsed = parse_bitcraze_text(text)

    result = {
        "source": "usb",
        "device": dev,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "base_station": {
            "serial": None,
            "channel": None,
            "model": "LH2",
        },
        "calibration": parsed["calibration"],
    }
    result["base_station"].update(parsed.get("base_station", {}))

    return normalize_lighthouse_factory_calibration(result)


def choose_output_path(args, calibration):
    if args.output:
        return Path(args.output)

    output_dir = Path(args.output_dir)
    serial = calibration.get("base_station", {}).get("serial")
    if serial:
        safe_serial = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(serial))
        filename = f"lighthouse_factory_calibration_{safe_serial}.json"
    else:
        safe_dev = re.sub(r"[^A-Za-z0-9_.-]+", "_", args.dev)
        filename = f"lighthouse_factory_calibration_{safe_dev}.json"

    return output_dir / filename


def save_calibration(path, calibration):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(calibration, f, indent=2)
        f.write("\n")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Lighthouse 2.0 factory calibration through USB using a Bitcraze-compatible reader."
    )
    parser.add_argument("--list-ports", action="store_true", help="List available serial ports and exit.")
    parser.add_argument("--dev", help="Serial device for the Lighthouse USB connection, for example COM3.")
    parser.add_argument("--output", help="Exact JSON output path.")
    parser.add_argument("--output-dir", help="Directory for lighthouse_factory_calibration_<serial>.json.")
    parser.add_argument(
        "--bitcraze-script",
        help="Path to Bitcraze crazyflie-firmware/tools/lighthouse/get_lh2_calib_data.py.",
    )
    parser.add_argument("--timeout", type=float, default=20.0, help="Read timeout in seconds.")
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        if args.list_ports:
            print_serial_ports()
            return 0

        if not args.dev:
            raise CalibrationReadError("Missing --dev. Use --list-ports to find the Lighthouse COM port.")
        if args.output and args.output_dir:
            raise CalibrationReadError("Use either --output or --output-dir, not both.")
        if not args.output and not args.output_dir:
            raise CalibrationReadError("Missing --output or --output-dir.")

        calibration = read_factory_calibration(
            dev=args.dev,
            bitcraze_script=args.bitcraze_script,
            timeout_s=args.timeout,
        )
        output_path = choose_output_path(args, calibration)
        save_calibration(output_path, calibration)

        serial = calibration["base_station"].get("serial") or "unknown serial"
        print(f"Saved Lighthouse factory calibration for {serial} to: {output_path}")
        return 0

    except CalibrationReadError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except ImportError as exc:
        print(f"ERROR: Missing dependency: {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"ERROR: OS/USB/permission error: {exc}", file=sys.stderr)
        return 2
    except ValueError as exc:
        print(f"ERROR: Invalid calibration data: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
