import json
import math
import cv2
import numpy as np

TICKS_PER_REV = 833333

layout = json.load(open("config/sensors_layout.json"))
origin = json.load(open("config/origin_angles.json"))

def lfsr_to_alpha_rad(lfsr):
    deg = (((lfsr % TICKS_PER_REV) / TICKS_PER_REV) * 120.0) - 60.0
    return math.radians(deg)

def alphas_to_theta_phi(alpha0, alpha1):
    theta = (alpha0 + alpha1) / 2.0
    numerator = math.sin(((alpha1 - alpha0) / 2.0) - (math.pi / 3.0))
    denominator = math.tan(math.pi / 6.0) * math.cos((alpha0 + alpha1) / 2.0)
    phi = math.atan2(numerator, denominator)
    return theta, phi

def angles_to_image_point(theta, phi):
    return [math.tan(theta), math.tan(phi) / math.cos(theta)]

def solve_for(bs, swap=False):
    obj = []
    img = []

    for sid, sinfo in layout["sensors"].items():
        m = origin["measurements"][str(bs)][sid]

        l0 = int(m["lfsr0_median"])
        l1 = int(m["lfsr1_median"])

        a0 = lfsr_to_alpha_rad(l0)
        a1 = lfsr_to_alpha_rad(l1)

        if swap:
            a0, a1 = a1, a0

        theta, phi = alphas_to_theta_phi(a0, a1)

        obj.append([sinfo["x"], sinfo["y"], sinfo["z"]])
        img.append(angles_to_image_point(theta, phi))

    obj = np.array(obj, dtype=np.float64)
    img = np.array(img, dtype=np.float64)
    K = np.eye(3, dtype=np.float64)
    dist = np.zeros((4, 1), dtype=np.float64)

    retval, rvecs, tvecs, reproj = cv2.solvePnPGeneric(
        obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE
    )

    print()
    print(f"BS {bs} | swap={swap} | solutions={len(rvecs)}")

    reproj_flat = np.array(reproj).reshape(-1)

    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        R, _ = cv2.Rodrigues(rvec)
        pos = (-R.T @ tvec).reshape(3)
        print(
            f"  sol {i}: "
            f"x={pos[0]:+.3f}, y={pos[1]:+.3f}, z={pos[2]:+.3f}, "
            f"reproj={float(reproj_flat[i]):.6f}"
        )

for bs in [4, 10]:
    solve_for(bs, swap=False)
    solve_for(bs, swap=True)
