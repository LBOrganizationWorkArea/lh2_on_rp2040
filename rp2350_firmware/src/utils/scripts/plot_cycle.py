#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_cycle.py — visualize one cycle of crossing_beams serial output.

Capture a cycle from the USB serial (any terminal will do) into a text file,
e.g.  `cat /dev/ttyACM2 > cycle.txt`  (Ctrl-C after a full lap), then:

    python plot_cycle.py cycle.txt

Parses the firmware's serial lines:
    A,<sensor>,<bs>,<horiz_deg>,<vert_deg>     per-sensor angle, per base station
    P,<sensor>,<x>,<y>,<z>                     per-sensor 3D point [m]
    C,<n>,<cx>,<cy>,<cz>                       centroid [m]   (frame delimiter)
(ANG / legend / other lines are ignored.)

Produces one figure with:
  1. Top-down XY path of the centroid, with detected corners marked and the
     4-sensor square drawn at each corner.
  2. 3D trajectory of the centroid + sensor squares at the corners.
  3. Angles vs frame (horiz & vert, BS0 & BS1, one line per sensor), with the
     corner frames marked.
"""

import argparse
import math
import sys

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d proj)
except ImportError:
    sys.exit("matplotlib is required:  pip install matplotlib")


# --------------------------------------------------------------------------- #
# Parsing
# --------------------------------------------------------------------------- #

class Frame:
    """One solve cycle: per-sensor points, centroid, per-sensor angles."""
    def __init__(self):
        self.pts = {}       # sensor -> (x, y, z)
        self.ang = {}       # (sensor, bs) -> (horiz, vert)
        self.centroid = None  # (cx, cy, cz)


def parse(path):
    frames = []
    cur = Frame()
    with open(path) as f:
        for line in f:
            line = line.strip()
            parts = line.split(",")
            tag = parts[0]
            try:
                if tag == "A" and len(parts) == 5:
                    s, bs = int(parts[1]), int(parts[2])
                    cur.ang[(s, bs)] = (float(parts[3]), float(parts[4]))
                elif tag == "P" and len(parts) == 5:
                    s = int(parts[1])
                    cur.pts[s] = (float(parts[2]), float(parts[3]), float(parts[4]))
                elif tag == "C" and len(parts) == 5:
                    cur.centroid = (float(parts[2]), float(parts[3]), float(parts[4]))
                    frames.append(cur)   # C closes the frame
                    cur = Frame()
            except ValueError:
                continue  # malformed numeric field — skip line
    # keep only frames that actually have a centroid
    return [fr for fr in frames if fr.centroid is not None]


# --------------------------------------------------------------------------- #
# Corner detection (data-driven, from the centroid XY heading changes)
# --------------------------------------------------------------------------- #

def detect_corners(cx, cy, turn_deg=40.0):
    """Return frame indices where the XY path changes heading sharply."""
    dirs = []  # (ux, uy, frame_index)
    for i in range(1, len(cx)):
        dx, dy = cx[i] - cx[i - 1], cy[i] - cy[i - 1]
        n = math.hypot(dx, dy)
        if n > 1e-6:
            dirs.append((dx / n, dy / n, i))

    corners = []
    for j in range(1, len(dirs)):
        d0, d1 = dirs[j - 1], dirs[j]
        dot = max(-1.0, min(1.0, d0[0] * d1[0] + d0[1] * d1[1]))
        if math.degrees(math.acos(dot)) > turn_deg:
            idx = d1[2]
            if not corners or idx - corners[-1] > 3:   # dedup nearby
                corners.append(idx)
    return corners


def square_xy(frame):
    """Closed-loop XY of the 4 sensor points, for drawing the body square."""
    order = [0, 1, 2, 3, 0]
    xs = [frame.pts[s][0] for s in order if s in frame.pts]
    ys = [frame.pts[s][1] for s in order if s in frame.pts]
    return xs, ys


# --------------------------------------------------------------------------- #
# Plotting
# --------------------------------------------------------------------------- #

def plot(frames):
    cx = [fr.centroid[0] for fr in frames]
    cy = [fr.centroid[1] for fr in frames]
    cz = [fr.centroid[2] for fr in frames]
    corners = detect_corners(cx, cy)

    fig = plt.figure(figsize=(14, 9))

    # --- 1. Top-down XY path -------------------------------------------------
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(cx, cy, "-o", ms=3, lw=1, color="tab:blue", label="centroid path")
    for k, ci in enumerate(corners):
        fr = frames[ci]
        ax1.plot(*fr.centroid[:2], "r*", ms=14)
        ax1.annotate(f"C{k}\n({fr.centroid[0]:.2f},{fr.centroid[1]:.2f})",
                     fr.centroid[:2], textcoords="offset points",
                     xytext=(8, 8), fontsize=8, color="red")
        sx, sy = square_xy(fr)
        ax1.plot(sx, sy, "-", color="green", lw=1.2, alpha=0.8)
    ax1.set_title("Top-down (XY) — centroid path + sensor square at corners")
    ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]")
    ax1.axis("equal"); ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)

    # --- 2. 3D trajectory ----------------------------------------------------
    ax2 = fig.add_subplot(2, 2, 2, projection="3d")
    ax2.plot(cx, cy, cz, "-", color="tab:blue", lw=1, label="centroid")
    for ci in corners:
        fr = frames[ci]
        ax2.scatter(*fr.centroid, color="red", s=60, marker="*")
        order = [0, 1, 2, 3, 0]
        xs = [fr.pts[s][0] for s in order if s in fr.pts]
        ys = [fr.pts[s][1] for s in order if s in fr.pts]
        zs = [fr.pts[s][2] for s in order if s in fr.pts]
        ax2.plot(xs, ys, zs, "-", color="green", lw=1.2)
    ax2.set_title("3D trajectory + sensor squares at corners")
    ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]"); ax2.set_zlabel("z [m]")
    ax2.legend(fontsize=8)

    # --- 3. Angles vs frame --------------------------------------------------
    ax3 = fig.add_subplot(2, 2, 3)  # horizontal
    ax4 = fig.add_subplot(2, 2, 4)  # vertical
    sensors = sorted({s for fr in frames for (s, _) in fr.ang})
    styles = {0: "-", 1: "--"}      # bs 0 solid, bs 1 dashed
    for s in sensors:
        for bs in (0, 1):
            fidx, hor, ver = [], [], []
            for i, fr in enumerate(frames):
                if (s, bs) in fr.ang:
                    h, v = fr.ang[(s, bs)]
                    fidx.append(i); hor.append(h); ver.append(v)
            if fidx:
                ax3.plot(fidx, hor, styles[bs], lw=1, label=f"S{s} BS{bs}")
                ax4.plot(fidx, ver, styles[bs], lw=1, label=f"S{s} BS{bs}")
    for ax, title, ylab in ((ax3, "Horizontal angle vs frame", "horiz [deg]"),
                            (ax4, "Vertical angle vs frame", "vert [deg]")):
        for ci in corners:
            ax.axvline(ci, color="red", ls=":", alpha=0.5)
        ax.set_title(title); ax.set_xlabel("frame"); ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3); ax.legend(fontsize=6, ncol=2)

    fig.suptitle(f"crossing_beams — {len(frames)} frames, "
                 f"{len(corners)} corners detected", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig("cycle_plot.png", dpi=130)
    print(f"Parsed {len(frames)} frames; corners at frames {corners}")
    print("Saved cycle_plot.png")
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", help="captured serial text file")
    args = ap.parse_args()

    frames = parse(args.logfile)
    if len(frames) < 2:
        sys.exit("Not enough frames parsed — need C lines with a full cycle.")
    plot(frames)


if __name__ == "__main__":
    main()
