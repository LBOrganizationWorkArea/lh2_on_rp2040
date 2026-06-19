#!/usr/bin/env python3
"""3D crossing-beam visualization built from the MEASURED serial angles.

For each frame we take the (horiz, vert) angles that came over serial, turn each
into a ray in the base station's frame, rotate it into the world, and draw it as
a real ray FROM the base station. We do NOT draw to the known point — the two
rays cross at the point on their own, which is the actual proof the angles
triangulate correctly. The measured point is plotted only as a reference marker.

Ray model (Bitcraze): d_local = normalize(1, tan(horiz), tan(vert)), local +X =
boresight; d_world = R · d_local. Both stations here use R = Ry(-90deg) (+Z up).
"""
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Base-station origins [m]; both look +Z, so R maps local +X -> world +Z.
BS_ORIGIN = {"BS0": (0.0, 0.0, 0.0), "BS1": (1.0, 0.0, 0.0)}
R = [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]   # Ry(-90deg)
RAY_LEN = 2.4   # metres to extend each ray (past the z=2 plane)

# One sample per frame, straight off the serial (sensor S0):
#   (label, BS0 h, BS0 v, BS1 h, BS1 v, measured P0)
FRAMES = [
    ("bottom", 0.00,  -5.99, 0.00, 21.55, (0.2100, 0.0000, 2.0001)),
    ("left",  23.99,  -0.00, 23.99, 26.56, (0.0000, 0.8900, 2.0000)),
    ("top",   26.57, -23.69, 26.57,  3.50, (0.8775, 1.0000, 2.0000)),
    ("right",  6.35, -26.57,  6.35, -0.00, (1.0000, 0.2225, 2.0000)),
]


def ray_dir(h_deg, v_deg):
    """Measured (horiz,vert) -> unit ray direction in WORLD frame."""
    h, v = math.radians(h_deg), math.radians(v_deg)
    dl = [1.0, math.tan(h), math.tan(v)]
    n = math.sqrt(sum(c * c for c in dl))
    dl = [c / n for c in dl]
    return [sum(R[i][j] * dl[j] for j in range(3)) for i in range(3)]


fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, projection="3d")

for name, (x, y, z) in BS_ORIGIN.items():
    ax.scatter(x, y, z, color="red", s=90, marker="^")
    ax.text(x, y, z, f"  {name}", color="red", fontsize=10)

colors = {"BS0": "tab:orange", "BS1": "tab:green"}
for label, h0, v0, h1, v1, P in FRAMES:
    # measured point (reference only — NOT used to build the rays)
    ax.scatter(*P, color="blue", s=55)
    ax.text(P[0], P[1], P[2], f"  {label}", color="blue", fontsize=8)

    for bs, (h, v) in (("BS0", (h0, v0)), ("BS1", (h1, v1))):
        o = BS_ORIGIN[bs]
        d = ray_dir(h, v)
        end = [o[i] + RAY_LEN * d[i] for i in range(3)]
        ax.plot([o[0], end[0]], [o[1], end[1]], [o[2], end[2]],
                color=colors[bs], lw=1.0, alpha=0.8)

# legend proxies
ax.plot([], [], color="tab:orange", label="BS0 ray (measured angle)")
ax.plot([], [], color="tab:green", label="BS1 ray (measured angle)")
ax.scatter([], [], color="blue", label="measured point (where rays cross)")
ax.legend(fontsize=8, loc="upper left")

ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
ax.set_title("Crossing beams from MEASURED angles\n(rays cast from serial, not drawn to point)")
fig.tight_layout()
fig.savefig("geometry_plot.png", dpi=130)
print("Saved geometry_plot.png")

# Numeric check: where does each measured ray pierce z=2, vs the reported point?
print("\nRay/point consistency (pierce z=2.0):")
for label, h0, v0, h1, v1, P in FRAMES:
    out = []
    for bs, (h, v) in (("BS0", (h0, v0)), ("BS1", (h1, v1))):
        o, d = BS_ORIGIN[bs], ray_dir(h, v)
        t = (2.0 - o[2]) / d[2]
        out.append((o[0] + t * d[0], o[1] + t * d[1]))
    print(f"  {label:6s}: BS0->({out[0][0]:.3f},{out[0][1]:.3f})  "
          f"BS1->({out[1][0]:.3f},{out[1][1]:.3f})  reported P0=({P[0]:.3f},{P[1]:.3f})")

plt.show()
