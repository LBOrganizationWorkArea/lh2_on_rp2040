#!/usr/bin/env python3

import runpy
import sys
from pathlib import Path


def add_default(argv, option, value):
    if option not in argv:
        argv.extend([option, value])


def main():
    here = Path(__file__).resolve().parent
    target = here / "04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py"

    forwarded = sys.argv[1:]
    add_default(forwarded, "--poses", "config/wand_calibration_poses_3d.json")
    add_default(forwarded, "--output", "config/lighthouse_geometry_wand_3d.json")
    add_default(forwarded, "--max-nfev", "600")

    sys.argv = [str(target)] + forwarded
    runpy.run_path(str(target), run_name="__main__")


if __name__ == "__main__":
    main()
