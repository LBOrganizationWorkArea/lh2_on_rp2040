import argparse
import time
import numpy as np

from lh2_2d_common import capture_image_point, apply_homography, load_json


def main():
    parser = argparse.ArgumentParser(description="Live 2D position from one Lighthouse homography.")
    parser.add_argument("--port", required=True, help="Example: COM3")
    parser.add_argument("--calibration", default="config/homography_2d.json")
    parser.add_argument("--duration", type=float, default=0.4)
    parser.add_argument("--alpha", type=float, default=0.35, help="Filter alpha. 1.0 = no filter.")
    args = parser.parse_args()

    calib = load_json(args.calibration)
    basestation = int(calib["basestation"])
    H = np.array(calib["H_image_to_world"], dtype=float)

    print("Live 2D position from Lighthouse homography")
    print("=" * 60)
    print(f"Calibration: {args.calibration}")
    print(f"Basestation: {basestation}")
    print(f"Port: {args.port}")
    print(f"Capture window: {args.duration} s")
    print(f"Filter alpha: {args.alpha}")
    print("=" * 60)
    print("Press Ctrl+C to stop.")
    print()

    xf = None
    yf = None

    while True:
        result = capture_image_point(
            port=args.port,
            basestation=basestation,
            duration=args.duration,
        )

        x, y = apply_homography(H, result["image_x"], result["image_y"])

        if xf is None:
            xf, yf = x, y
        else:
            a = args.alpha
            xf = a * x + (1.0 - a) * xf
            yf = a * y + (1.0 - a) * yf

        print(
            f"x={xf:+.3f} m | y={yf:+.3f} m | "
            f"raw=({x:+.3f},{y:+.3f}) | "
            f"sensors={result['visible_sensors']}"
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped.")
