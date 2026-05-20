import argparse
from collections import Counter

from mocap_lh2 import load_lh2_csv, load_mocap_csv


def main():
    parser = argparse.ArgumentParser(description="Check LH2 and mocap recordings before calibration.")
    parser.add_argument("--lh2", default="data/lh2_record.csv")
    parser.add_argument("--mocap", default="data/mocap.csv")
    args = parser.parse_args()

    lh2 = load_lh2_csv(args.lh2)
    mocap = load_mocap_csv(args.mocap)

    if not lh2:
        raise SystemExit("LH2 file has no usable rows.")
    if not mocap:
        raise SystemExit("Mocap file has no usable rows.")

    lh2_t0 = min(row["pc_time_s"] for row in lh2)
    lh2_t1 = max(row["pc_time_s"] for row in lh2)
    mocap_t0 = min(row["pc_time_s"] for row in mocap)
    mocap_t1 = max(row["pc_time_s"] for row in mocap)

    overlap0 = max(lh2_t0, mocap_t0)
    overlap1 = min(lh2_t1, mocap_t1)
    overlap = max(0.0, overlap1 - overlap0)

    bs_counts = Counter(row["basestation"] for row in lh2)
    sensor_counts = Counter(row["sensor"] for row in lh2)
    sweep_counts = Counter(row["sweep"] for row in lh2)
    poly_counts = Counter(row["polynomial"] for row in lh2)

    print("=" * 70)
    print("Recording check")
    print(f"LH2 rows:   {len(lh2)}")
    print(f"Mocap rows: {len(mocap)}")
    print(f"LH2 time:   {lh2_t0:.3f} -> {lh2_t1:.3f}")
    print(f"Mocap time: {mocap_t0:.3f} -> {mocap_t1:.3f}")
    print(f"Overlap:    {overlap:.3f} s")
    print(f"Base stations: {dict(sorted(bs_counts.items()))}")
    print(f"Sensors:       {dict(sorted(sensor_counts.items()))}")
    print(f"Sweeps:        {dict(sorted(sweep_counts.items()))}")
    print(f"Polynomials:   {dict(sorted(poly_counts.items()))}")
    print("=" * 70)

    if overlap <= 1.0:
        print("WARNING: little or no time overlap. Calibration will not work unless timestamps are aligned.")


if __name__ == "__main__":
    main()
