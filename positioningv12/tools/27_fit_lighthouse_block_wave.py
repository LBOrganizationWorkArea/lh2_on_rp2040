#!/usr/bin/env python3

import os
import sys
from pathlib import Path


def add_default(argv, option, value):
    if option not in argv:
        argv.extend([option, value])


def main():
    here = Path(__file__).resolve().parent
    target = here / "22_fit_relative_lighthouse_frame.py"

    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ.setdefault(name, "1")

    forwarded = sys.argv[1:]
    add_default(forwarded, "--wave", "config/lh2a_wave_record.json")
    add_default(forwarded, "--output", "config/lighthouse_block_from_wave.json")
    add_default(forwarded, "--max-family-spread-deg", "0.6")
    add_default(forwarded, "--min-channels", "12")
    add_default(forwarded, "--max-frames", "25")
    add_default(forwarded, "--bs10-guess", "1.4,0,0")
    add_default(forwarded, "--bs-distance-prior", "1.4")
    add_default(forwarded, "--bs-distance-sigma", "0.05")
    add_default(forwarded, "--bs-distance-weight", "200")
    add_default(forwarded, "--starts", "2")
    add_default(forwarded, "--workers", "8")
    add_default(forwarded, "--max-nfev", "80")
    add_default(forwarded, "--convention-search", "all")

    os.execv(sys.executable, [sys.executable, str(target)] + forwarded)


if __name__ == "__main__":
    main()
