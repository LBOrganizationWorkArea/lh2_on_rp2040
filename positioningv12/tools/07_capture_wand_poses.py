#!/usr/bin/env python3

import runpy
import sys
from pathlib import Path


def add_default(argv, option, value):
    if option not in argv:
        argv.extend([option, value])


def main():
    here = Path(__file__).resolve().parent
    target = here / "03_capture_calibration_poses.py"

    forwarded = sys.argv[1:]
    add_default(forwarded, "--pose-file", "config/wand_3d_points.json")
    add_default(forwarded, "--output", "config/wand_calibration_poses_3d.json")
    add_default(forwarded, "--min-observations", "8")
    add_default(forwarded, "--min-sensors", "2")
    add_default(forwarded, "--min-basestations", "1")

    sys.argv = [str(target)] + forwarded
    runpy.run_path(str(target), run_name="__main__")


if __name__ == "__main__":
    main()
