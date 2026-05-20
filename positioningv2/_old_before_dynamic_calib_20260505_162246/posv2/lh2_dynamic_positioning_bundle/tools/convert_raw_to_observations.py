#!/usr/bin/env python3
"""
Convert raw serial JSONL to angle observation CSV.

Expected easiest firmware line formats accepted:
1) JSON line:
   {"sensor_id":"S0","lighthouse_id":"4","theta":0.123,"phi":-0.045}
2) CSV-like line:
   OBS,S0,4,0.123,-0.045
3) Key-value line:
   sensor=S0 lh=4 theta=0.123 phi=-0.045

If your firmware prints only raw LFSR/polynomial data like:
   S0 GP10/11: [3-37935] [5-71415] ...
then theta/phi are not directly present; adapt this converter after the angle decoder is ready.
"""
import argparse
import csv
import json
import re
from pathlib import Path

KV_RE = re.compile(r"sensor=(S\d+)\s+lh=(\d+)\s+theta=([-+0-9.eE]+)\s+phi=([-+0-9.eE]+)")
CSV_RE = re.compile(r"OBS\s*,\s*(S\d+)\s*,\s*(\d+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)")


def parse_line(raw_line):
    raw_line = raw_line.strip()
    if not raw_line:
        return None

    # JSON emitted directly by firmware
    if raw_line.startswith("{"):
        try:
            obj = json.loads(raw_line)
            sid = str(obj.get("sensor_id") or obj.get("sensor") or "")
            lh = str(obj.get("lighthouse_id") or obj.get("lh") or "")
            theta = obj.get("theta")
            phi = obj.get("phi")
            if sid and lh and theta is not None and phi is not None:
                return sid, lh, float(theta), float(phi)
        except Exception:
            return None

    m = CSV_RE.search(raw_line)
    if m:
        sid, lh, theta, phi = m.groups()
        return sid, lh, float(theta), float(phi)

    m = KV_RE.search(raw_line)
    if m:
        sid, lh, theta, phi = m.groups()
        return sid, lh, float(theta), float(phi)

    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    raw_path = Path(args.raw)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    total = 0
    parsed = 0
    with raw_path.open("r", encoding="utf-8") as fin, out_path.open("w", newline="", encoding="utf-8") as fout:
        writer = csv.DictWriter(fout, fieldnames=["timestamp", "sensor_id", "lighthouse_id", "theta", "phi", "valid"])
        writer.writeheader()
        for line in fin:
            total += 1
            try:
                rec = json.loads(line)
                ts = float(rec.get("timestamp_pc", 0.0))
                raw = rec.get("raw", "")
            except Exception:
                ts = 0.0
                raw = line

            obs = parse_line(raw)
            if obs is None:
                continue
            sid, lh, theta, phi = obs
            writer.writerow({"timestamp": ts, "sensor_id": sid, "lighthouse_id": lh, "theta": theta, "phi": phi, "valid": 1})
            parsed += 1

    print(f"Read {total} raw lines")
    print(f"Parsed {parsed} angle observations")
    print(f"Saved: {out_path}")
    if parsed == 0:
        print("WARNING: no theta/phi observations found. Your firmware output probably needs an angle decoder/export format.")


if __name__ == "__main__":
    main()
