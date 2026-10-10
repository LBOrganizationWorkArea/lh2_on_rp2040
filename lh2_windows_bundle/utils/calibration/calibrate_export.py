#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_export.py — turn calibrated lighthouse geometry into a firmware header.

Single source of truth for base-station poses used by the ray-crossing solver
(see rp2350_firmware/src/crossing_beams/RAY_CROSSING_PLAN.md).

Two modes, identical output format:

  --synthetic        Use a known, hardcoded two-base-station geometry. Runs today
                     with only numpy — no hardware, no lbees imports. Used to
                     develop/test the firmware ray-crossing path.

  (default / real)   Read calibrated poses produced by the real pipeline
                     (calibrate_cli.py -> _estimate_geometry). Loads either a
                     Bitcraze-style geometry YAML (geos: position + rotation_quat)
                     or, if available, runs the pipeline on a measurements file.

Output: a C header (bs_poses_cal.h) containing:

    static const lh2_bs_pose_t BS_POSES[NUM_BS] = { {origin, R}, {origin, R} };

R is the base-station-local -> world rotation matrix, computed here in Python so
the quaternion convention never has to be reimplemented on the microcontroller.

World frame: origin at the calibration origin, +X toward the x-axis sample,
metres (scale fixed by the reference distance). The same convention the firmware
assumes (BS-local +X = boresight).
"""

import argparse
import math
import sys

import numpy as np

# Physical base-station ids behind each firmware base-station index, for comments.
BS_INDEX_NOTE = {
    0: "poly 8/9,  BS4",
    1: "poly 20/21, BS10",
}


# --------------------------------------------------------------------------- #
# Pose container
# --------------------------------------------------------------------------- #

class BsPose:
    """A base station pose in the world frame: origin [m] + R (local->world)."""

    def __init__(self, origin, R):
        self.origin = np.asarray(origin, dtype=float).reshape(3)
        self.R = np.asarray(R, dtype=float).reshape(3, 3)

    @classmethod
    def from_quat_xyzw(cls, origin, quat_xyzw):
        """Build from a scipy-convention [x, y, z, w] quaternion."""
        from scipy.spatial.transform import Rotation
        R = Rotation.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()
        return cls(origin, R)


# --------------------------------------------------------------------------- #
# Synthetic geometry (must match the firmware synthetic injector)
# --------------------------------------------------------------------------- #

def _roty(deg):
    """Rotation about the world Y axis."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def synthetic_poses():
    """
    Two base stations mounted on the ceiling (2.26 m apart along X, at z=3.45 m),
    both looking straight down (-Z). R = Ry(+90deg) maps the BS-local boresight
    (+X) to world -Z.

    With this geometry a body at world (bx, by, 2) is seen by both stations and
    the two rays cross exactly at it (baseline along X is perpendicular to the
    -Z viewing direction -> well-conditioned).
    """
    R = _roty(90.0)   # local +X -> world -Z
    return [
        BsPose(origin=[0.0,  0.0, 3.45], R=R),   # BS0  (geos:0)
        BsPose(origin=[2.26, 0.0, 3.45], R=R),   # BS1  (geos:1)
    ]


# --------------------------------------------------------------------------- #
# Real geometry: load a Bitcraze-style YAML (geos: position + rotation_quat)
# --------------------------------------------------------------------------- #

def poses_from_yaml(path):
    """
    Load poses from a lighthouse_system_configuration YAML (test_output.yaml
    format). rotation_quat is taken as scipy [x, y, z, w] — matching
    lighthouse_geometry_types.py which feeds it straight to Rotation.from_quat.
    """
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)
    geos = data["geos"]
    poses = []
    for idx in sorted(geos.keys(), key=int):
        g = geos[idx]
        poses.append(BsPose.from_quat_xyzw(g["position"], g["rotation_quat"]))
    return poses


# --------------------------------------------------------------------------- #
# Reporting + header generation
# --------------------------------------------------------------------------- #

def print_summary(poses, source):
    print(f"# calibrate_export — source: {source}")
    print("World frame: origin=(0,0,0), +X -> x-axis sample, metres "
          "(scale = reference distance).")
    print("BS-local +X = boresight.\n")
    for i, p in enumerate(poses):
        note = BS_INDEX_NOTE.get(i, "")
        o = p.origin
        boresight = p.R @ np.array([1.0, 0.0, 0.0])  # world dir the BS faces
        print(f"BS{i} ({note}):")
        print(f"    position : [{o[0]:.4f}, {o[1]:.4f}, {o[2]:.4f}] m")
        print(f"    boresight: R*(1,0,0) = "
              f"[{boresight[0]:+.3f}, {boresight[1]:+.3f}, {boresight[2]:+.3f}]")
    print()


def _fmt_row(row):
    return "{" + ", ".join(f"{v:.6f}f" for v in row) + "}"


def write_header(poses, source, out_path):
    lines = []
    lines.append("/* bs_poses_cal.h — GENERATED by calibrate_export.py. DO NOT hand-edit. */")
    lines.append("/*")
    lines.append(" * World frame: origin at calibration origin, +X -> x-axis sample, metres.")
    lines.append(" * BS-local +X = boresight. R is base-station-local -> world.")
    lines.append(" */")
    lines.append("#ifndef BS_POSES_CAL_H")
    lines.append("#define BS_POSES_CAL_H")
    lines.append("")
    lines.append('#include "solve3d/solve3d.h"   /* lh2_bs_pose_t, NUM_BS */')
    lines.append("")
    lines.append(f'#define BS_POSE_SOURCE "{source}"')
    lines.append("")
    lines.append("static const lh2_bs_pose_t BS_POSES[NUM_BS] = {")
    for i, p in enumerate(poses):
        note = BS_INDEX_NOTE.get(i, "")
        o = p.origin
        lines.append(f"    {{  /* BS{i}  ({note}) */")
        lines.append(f"        .origin = {{{o[0]:.6f}f, {o[1]:.6f}f, {o[2]:.6f}f}},")
        lines.append(f"        .R = {{ {_fmt_row(p.R[0])},")
        lines.append(f"               {_fmt_row(p.R[1])},")
        lines.append(f"               {_fmt_row(p.R[2])} }},")
        lines.append("    },")
    lines.append("};")
    lines.append("")
    lines.append("#endif /* BS_POSES_CAL_H */")

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {out_path}  ({len(poses)} base stations)")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--synthetic", action="store_true",
                    help="use the hardcoded synthetic geometry (no hardware)")
    ap.add_argument("--yaml",
                    help="load real poses from a lighthouse geometry YAML")
    ap.add_argument("-o", "--output", default="bs_poses_cal.h",
                    help="output C header path (default: bs_poses_cal.h)")
    args = ap.parse_args()

    if args.synthetic:
        poses, source = synthetic_poses(), "synthetic"
    elif args.yaml:
        poses, source = poses_from_yaml(args.yaml), f"yaml:{args.yaml}"
    else:
        ap.error("choose --synthetic or --yaml <file> "
                 "(real measurements->poses pipeline runs via calibrate_cli.py)")

    if len(poses) < 2:
        ap.error(f"need at least 2 base stations, got {len(poses)}")

    print_summary(poses, source)
    write_header(poses, source, args.output)


if __name__ == "__main__":
    main()
