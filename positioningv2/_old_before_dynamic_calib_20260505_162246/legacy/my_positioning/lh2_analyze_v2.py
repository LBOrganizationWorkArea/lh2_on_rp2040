from pathlib import Path
import pandas as pd

WORK_DIR = Path(__file__).resolve().parent
DATA_DIR = WORK_DIR / "data_clean"

FILES = [
    ("v2_horizontal_left",   "horizontal", 0.70, 1.00),
    ("v2_horizontal_center", "horizontal", 1.00, 1.00),
    ("v2_horizontal_right",  "horizontal", 1.30, 1.00),
    ("v2_depth_near",       "depth",      1.00, 0.60),
    ("v2_depth_center",     "depth",      1.00, 1.00),
    ("v2_depth_far",        "depth",      1.00, 1.60),
]

CHANNELS = [
    (4, 0),
    (4, 1),
    (10, 0),
    (10, 1),
]


def median_channel(df, basestation, sweep):
    values = df[
        (df["sensor"] == 2)
        & (df["basestation"] == basestation)
        & (df["sweep"] == sweep)
    ]["lfsr_location"]

    if len(values) == 0:
        return None

    return float(values.median())


def main():
    print("V2 calibration analysis")
    print("=" * 110)

    results = []

    for file_name, test_type, x, y in FILES:
        path = DATA_DIR / f"{file_name}.csv"

        if not path.exists():
            print()
            print(f"{file_name}: ERROR file not found")
            continue

        df = pd.read_csv(path)

        row = {
            "file": file_name,
            "type": test_type,
            "x": x,
            "y": y,
        }

        for basestation, sweep in CHANNELS:
            row[f"LH{basestation}_s{sweep}"] = median_channel(df, basestation, sweep)

        results.append(row)

    for row in results:
        print()
        print(f"{row['file']}  |  type={row['type']}  |  x={row['x']:.2f}, y={row['y']:.2f}")
        print(f"  Lighthouse_4  sweep_0 = {row['LH4_s0']}")
        print(f"  Lighthouse_4  sweep_1 = {row['LH4_s1']}")
        print(f"  Lighthouse_10 sweep_0 = {row['LH10_s0']}")
        print(f"  Lighthouse_10 sweep_1 = {row['LH10_s1']}")

    print()
    print("=" * 110)
    print("Compact table:")
    print("file,x,y,LH4_s0,LH4_s1,LH10_s0,LH10_s1")

    for row in results:
        print(
            f"{row['file']},"
            f"{row['x']:.2f},"
            f"{row['y']:.2f},"
            f"{row['LH4_s0']},"
            f"{row['LH4_s1']},"
            f"{row['LH10_s0']},"
            f"{row['LH10_s1']}"
        )


if __name__ == "__main__":
    main()