#!/usr/bin/env python3

import os
import sys
from pathlib import Path


def add_default(argv, option, value):
    if option not in argv:
        argv.extend([option, value])


def main():
    here = Path(__file__).resolve().parent
    target = here / "04_estimate_lighthouse_geometry_lh2_guided_ultrafast.py"

    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ.setdefault(name, "1")

    forwarded = sys.argv[1:]
    add_default(forwarded, "--poses", "config/wand_calibration_poses_3d_lh2a_families.json")
    add_default(forwarded, "--output", "config/lighthouse_geometry_lh2a_families.json")
    if "--prefer-raw-angles" not in forwarded:
        forwarded.append("--prefer-raw-angles")
    add_default(forwarded, "--coarse-nfev", "120")
    add_default(forwarded, "--refine-top-k", "16")
    add_default(forwarded, "--max-nfev", "600")

    os.execv(sys.executable, [sys.executable, str(target)] + forwarded)


if __name__ == "__main__":
    main()
