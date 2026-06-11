#!/usr/bin/env python3

import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="v12 placeholder for live PnP/angle pose in the Lighthouse frame.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--geometry", default="config/lighthouse_relative_geometry.json")
    parser.add_argument("--anchor", default="config/lighthouse_to_room_transform.json")
    args = parser.parse_args()

    missing = [str(path) for path in (Path(args.geometry), Path(args.anchor)) if not path.exists()]
    if missing:
        raise SystemExit("Missing calibration file(s): " + ", ".join(missing))

    print("=" * 88)
    print("v12 live PnP")
    print(f"Port: {args.port}")
    print(f"Geometry: {args.geometry}")
    print(f"Anchor:   {args.anchor}")
    print("=" * 88)
    print("This entry point is ready, but live PnP is not implemented yet.")
    print("The intended loop reads LH2A angles, solves drone pose in the BS4 frame,")
    print("then transforms that pose into the room frame.")


if __name__ == "__main__":
    main()
