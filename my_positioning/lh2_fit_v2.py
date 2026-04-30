from pathlib import Path
import json
import numpy as np
import pandas as pd

WORK_DIR = Path(__file__).resolve().parent
DATA_DIR = WORK_DIR / "data_clean"
MODEL_FILE = WORK_DIR / "lh2_model_v2.json"

FILES = [
    ("v2_horizontal_left",   0.70, 1.00),
    ("v2_horizontal_center", 1.00, 1.00),
    ("v2_horizontal_right",  1.30, 1.00),
    ("v2_depth_near",       1.00, 0.70),
    ("v2_depth_center",     1.00, 1.00),
    ("v2_depth_far",        1.00, 1.30),
]


def median_value(df, basestation, sweep):
    values = df[
        (df["sensor"] == 2)
        & (df["basestation"] == basestation)
        & (df["sweep"] == sweep)
    ]["lfsr_location"]

    if len(values) == 0:
        return None

    return float(values.median())


def extract_features(file_name):
    path = DATA_DIR / f"{file_name}.csv"
    df = pd.read_csv(path)

    lh4_s0 = median_value(df, 4, 0)
    lh4_s1 = median_value(df, 4, 1)
    lh10_s0 = median_value(df, 10, 0)
    lh10_s1 = median_value(df, 10, 1)

    if None in [lh4_s0, lh4_s1, lh10_s0, lh10_s1]:
        raise RuntimeError(f"Missing values in {file_name}")

    lh4_value = min(lh4_s0, lh4_s1)
    lh10_value = min(lh10_s0, lh10_s1)

    return [lh4_value, lh10_value]


def main():
    X = []
    Yx = []
    Yy = []

    print("V2 selected features")
    print("=" * 80)

    for file_name, x, y in FILES:
        lh4_value, lh10_value = extract_features(file_name)

        X.append([1.0, lh4_value, lh10_value])
        Yx.append(x)
        Yy.append(y)

        print()
        print(file_name)
        print(f"  real position: x={x:.2f}, y={y:.2f}")
        print(f"  selected LH4  value = {lh4_value}")
        print(f"  selected LH10 value = {lh10_value}")

    X = np.array(X, dtype=float)
    Yx = np.array(Yx, dtype=float)
    Yy = np.array(Yy, dtype=float)

    mean = X[:, 1:].mean(axis=0)
    std = X[:, 1:].std(axis=0)
    std[std == 0] = 1.0

    Xn = X.copy()
    Xn[:, 1:] = (Xn[:, 1:] - mean) / std

    coeff_x, *_ = np.linalg.lstsq(Xn, Yx, rcond=None)
    coeff_y, *_ = np.linalg.lstsq(Xn, Yy, rcond=None)

    pred_x = Xn @ coeff_x
    pred_y = Xn @ coeff_y

    print()
    print("=" * 80)
    print("Reconstruction")
    print("=" * 80)

    errors = []

    for i, (file_name, real_x, real_y) in enumerate(FILES):
        px = pred_x[i]
        py = pred_y[i]
        err = np.sqrt((px - real_x) ** 2 + (py - real_y) ** 2)
        errors.append(err)

        print(
            f"{file_name:22s} | "
            f"real=({real_x:.2f}, {real_y:.2f}) | "
            f"pred=({px:.2f}, {py:.2f}) | "
            f"error={err * 100:.1f} cm"
        )

    model = {
        "method": "min_sweep_per_lighthouse",
        "mean": mean.tolist(),
        "std": std.tolist(),
        "coeff_x": coeff_x.tolist(),
        "coeff_y": coeff_y.tolist(),
    }

    with open(MODEL_FILE, "w") as f:
        json.dump(model, f, indent=2)

    print()
    print(f"Mean error = {np.mean(errors) * 100:.1f} cm")
    print(f"Model saved to: {MODEL_FILE}")


if __name__ == "__main__":
    main()