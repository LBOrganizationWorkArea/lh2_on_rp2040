#!/usr/bin/env python3
"""Basic filter for dynamic Lighthouse angle observations."""
import argparse
from pathlib import Path
import pandas as pd
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--max_abs_angle", type=float, default=1.45, help="Radians; reject too extreme theta/phi")
    ap.add_argument("--frame_ms", type=float, default=20.0)
    ap.add_argument("--min_sensors", type=int, default=3)
    ap.add_argument("--min_lighthouses", type=int, default=2)
    args = ap.parse_args()

    df = pd.read_csv(args.input)
    before = len(df)
    df = df[df.get("valid", 1).astype(int) == 1].copy()
    df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=["timestamp", "sensor_id", "lighthouse_id", "theta", "phi"])
    df["sensor_id"] = df["sensor_id"].astype(str)
    df["lighthouse_id"] = df["lighthouse_id"].astype(str)
    df = df[(df["theta"].abs() <= args.max_abs_angle) & (df["phi"].abs() <= args.max_abs_angle)]

    frame_s = args.frame_ms / 1000.0
    df["frame_id"] = (df["timestamp"] / frame_s).round().astype(int)

    keep_frames = []
    for fid, g in df.groupby("frame_id"):
        if g["sensor_id"].nunique() >= args.min_sensors and g["lighthouse_id"].nunique() >= args.min_lighthouses:
            keep_frames.append(fid)
    df = df[df["frame_id"].isin(keep_frames)].copy()

    # Average duplicate observations in same frame/sensor/lighthouse
    df = df.groupby(["frame_id", "sensor_id", "lighthouse_id"], as_index=False).agg({
        "timestamp": "mean",
        "theta": "median",
        "phi": "median",
        "valid": "max"
    })

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)
    print(f"Input rows: {before}")
    print(f"Filtered rows: {len(df)}")
    print(f"Frames kept: {df['frame_id'].nunique() if len(df) else 0}")
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
