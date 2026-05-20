import argparse
import json
import numpy as np

from lh2_2d_common import capture_image_point, compute_homography, save_json


CALIBRATION_POINTS = [
    ("P0_center", 0.0, 0.0),
    ("P1_right_30cm", -0.3, 0.0),
    ("P2_left_30cm", 0.3, 0.0),
    ("P3_up_30cm", 0.0, 0.3),
    ("P4_down_30cm", 0.0, -0.3),
]


def main():
    parser = argparse.ArgumentParser(description="Calibrate 2D position from one Lighthouse using homography.")
    parser.add_argument("--port", required=True, help="Example: COM3")
    parser.add_argument("--basestation", type=int, required=True, help="Example: 4 or 10")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--output", default="config/homography_2d.json")
    args = parser.parse_args()

    print("2D Homography calibration")
    print("=" * 60)
    print(f"Basestation: {args.basestation}")
    print(f"Port: {args.port}")
    print(f"Duration per point: {args.duration} s")
    print("=" * 60)
    print("IMPORTANT: keep drone orientation fixed for all points.")
    print()

    image_points = []
    world_points = []
    records = []

    for name, x, y in CALIBRATION_POINTS:
        print("=" * 60)
        print(f"Place drone at: {name}")
        print(f"World position: x={x:+.3f} m, y={y:+.3f} m")
        print("Keep it still.")
        input("Press ENTER when ready...")

        print("Capturing...")
        result = capture_image_point(
            port=args.port,
            basestation=args.basestation,
            duration=args.duration,
        )

        ix = result["image_x"]
        iy = result["image_y"]

        print(f"Captured image point: u={ix:+.6f}, v={iy:+.6f}")
        print(f"Visible sensors: {result['visible_sensors']}")

        image_points.append((ix, iy))
        world_points.append((x, y))

        records.append({
            "name": name,
            "world_x_m": x,
            "world_y_m": y,
            "image_x": ix,
            "image_y": iy,
            "visible_sensors": result["visible_sensors"],
        })

    H = compute_homography(image_points, world_points)

    output = {
        "method": "single_lighthouse_2d_homography",
        "basestation": args.basestation,
        "points": records,
        "H_image_to_world": H.tolist(),
    }

    save_json(args.output, output)

    print()
    print("=" * 60)
    print(f"Saved calibration: {args.output}")
    print("H image -> world:")
    print(np.array(H))
    print("=" * 60)


if __name__ == "__main__":
    main()
