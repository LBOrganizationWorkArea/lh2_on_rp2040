import argparse
import math
import numpy as np

from lh2_2d_common import capture_image_point, apply_homography, load_json


def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    calib = load_json(args.calibration)
    bs = int(calib["basestation"])
    H = np.array(calib["H_image_to_world"], dtype=float)

    print("Check current position against calibration points")
    print("=" * 60)
    print(f"Calibration: {args.calibration}")
    print(f"Basestation: {bs}")
    print("Keep drone still.")
    print("=" * 60)

    current = capture_image_point(
        port=args.port,
        basestation=bs,
        duration=args.duration,
    )

    cu = current["image_x"]
    cv = current["image_y"]
    x, y = apply_homography(H, cu, cv)

    print()
    print(f"Current image point: u={cu:+.8f}, v={cv:+.8f}")
    print(f"Current world pos:   x={x:+.4f} m, y={y:+.4f} m")
    print(f"Visible sensors:     {current['visible_sensors']}")
    print()

    rankings = []

    for p in calib["points"]:
        pu = float(p["image_x"])
        pv = float(p["image_y"])
        d = dist((cu, cv), (pu, pv))

        rankings.append((d, p))

    rankings.sort(key=lambda e: e[0])

    print("Nearest calibration image points:")
    for d, p in rankings:
        print(
            f"{p['name']:15s} | "
            f"world=({p['world_x_m']:+.3f},{p['world_y_m']:+.3f}) | "
            f"image=({p['image_x']:+.8f},{p['image_y']:+.8f}) | "
            f"d_image={d:.8f}"
        )


if __name__ == "__main__":
    main()
