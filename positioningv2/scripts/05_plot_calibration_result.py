import argparse
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from dynamic_lh2_common import load_json


def main():
    parser = argparse.ArgumentParser(description="Plot top-view dynamic Lighthouse calibration result.")
    parser.add_argument("--geometry", default="config/lighthouse_geometry.json")
    parser.add_argument("--output", default="data/logs/calibration_result.png")
    args = parser.parse_args()

    geometry = load_json(args.geometry)
    trajectory = geometry.get("estimated_trajectory", [])

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("Dynamic LH2 calibration top view")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")

    if trajectory:
        xs = [item["x"] for item in trajectory]
        ys = [item["y"] for item in trajectory]
        ax.plot(xs, ys, "-", color="tab:blue", alpha=0.75, label="estimated drone path")
        ax.scatter(xs[0], ys[0], color="tab:green", s=60, label="start")
        ax.scatter(xs[-1], ys[-1], color="tab:red", s=60, label="end")

    for item in geometry.get("lighthouses", []):
        translation = np.asarray(item["translation"], dtype=float)
        rot = Rotation.from_rotvec(item["rotation_vector"]).as_matrix()
        forward = rot[:, 0]
        ax.scatter([translation[0]], [translation[1]], marker="^", s=120, label=f"BS{item['id']}")
        ax.arrow(
            translation[0],
            translation[1],
            forward[0] * 0.4,
            forward[1] * 0.4,
            width=0.015,
            length_includes_head=True,
            alpha=0.8,
        )
        ax.text(translation[0], translation[1], f" BS{item['id']}", va="bottom")

    ax.legend(loc="best")
    fig.tight_layout()
    plt.savefig(args.output, dpi=160)
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
